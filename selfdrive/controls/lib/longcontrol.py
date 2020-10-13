from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import PIDController

LongCtrlState = log.ControlsState.LongControlState

STOPPING_EGO_SPEED = 0.2
MIN_CAN_SPEED = 0.3  # TODO: parametrize this in car interface
STOPPING_TARGET_SPEED = MIN_CAN_SPEED + 0.01
STARTING_TARGET_SPEED = 0.01
BRAKE_THRESHOLD_TO_PID = 1.0

STOPPING_BRAKE_RATE = 0.2  # brake_travel/s while trying to stop
STARTING_BRAKE_RATE = 5.  # brake_travel/s while releasing on restart
BRAKE_STOPPING_TARGET = 1.2 # apply at least this amount of brake to maintain the vehicle stationary

_MAX_SPEED_ERROR_BP = [0., 30.]  # speed breakpoints
_MAX_SPEED_ERROR_V = [1.5, .8]  # max positive v_pid error VS actual speed; this avoids controls windup due to slow pedal resp

RATE = 100.0


def long_control_state_trans(active, long_control_state, v_ego, v_target, output_gb, standstill, stop, vlead):
  """Update longitudinal control state machine"""

  starting_condition = (vlead > STARTING_TARGET_SPEED or v_target > STARTING_TARGET_SPEED) and not stop
  stopping_condition = stop or (v_ego < 2.0 and standstill and not starting_condition)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if output_gb >= 0:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP, compute_gb):
    self.long_control_state = LongCtrlState.off  # initialized to off

    kdBP = [0., 16., 35.]
    kdV = [0.08, 0.215, 0.51]

    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             (CP.longitudinalTuning.kfBP, CP.longitudinalTuning.kfV),
                             (kdBP, kdV),
                             rate=RATE,
                             sat_limit=0.8,
                             convert=compute_gb)
    self.v_pid = 0.0
    self.last_output_gb = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid
    self.stop = False
    self.stop_timer = 0

  def update(self, active, CS, v_target, v_target_future, a_target, CP, hasLead, radarState):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Actuation limits
    gas_max = interp(CS.vEgo, CP.gasMaxBP, CP.gasMaxV)
    brake_max = interp(CS.vEgo, CP.brakeMaxBP, CP.brakeMaxV)

    # Update state machine
    output_gb = self.last_output_gb
    if radarState is None:
      dRel = 200
      vLead = 0
    else:
      dRel = radarState.leadOne.dRel
      vLead = radarState.leadOne.vLead
    if hasLead and dRel < 5. and radarState.leadOne.status:
      self.stop = True
      if self.stop:
        self.stop_timer = 100
    elif self.stop_timer > 0:
      self.stop_timer -= 1
    else:
      self.stop = False
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo, v_target,
                                                       output_gb, CS.standstill, self.stop, vLead)

    v_ego_pid = max(CS.vEgo, MIN_CAN_SPEED)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off or CS.gasPressed:
      self.v_pid = v_ego_pid
      self.pid.reset()
      output_gb = 0.

    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target
      self.pid.pos_limit = gas_max
      self.pid.neg_limit = - brake_max

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7
      deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)

      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target,
                                  freeze_integrator=prevent_overshoot, leadvisible=hasLead, leaddistance=dRel, leadvel=vLead)
      if prevent_overshoot:
        output_gb = min(output_gb, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      factor = 1.
      if hasLead:
        factor = interp(dRel, [2., 3., 4., 5., 6., 7., 8.], [5., 2.5, 1., .5, .25, .05, .005])
      if output_gb > -BRAKE_STOPPING_TARGET:
        output_gb -= (STOPPING_BRAKE_RATE * factor) / RATE
      if output_gb < -.5 and CS.standstill:
        output_gb += .033
      output_gb = clip(output_gb, -brake_max, gas_max)
      self.v_pid = CS.vEgo
      self.pid.reset()

    # Intention is to move again, release brake fast before handing control to PID
    elif self.long_control_state == LongCtrlState.starting:
      factor = 1.
      if hasLead:
        factor = interp(dRel, [0., 2., 4., 6.], [2., 2., 2., 3.])
      if output_gb < 2.:
        output_gb += (STARTING_BRAKE_RATE * factor) / RATE
      self.v_pid = CS.vEgo
      self.pid.reset()

    self.last_output_gb = output_gb
    final_gas = clip(output_gb, 0., gas_max)
    final_brake = -clip(output_gb, -brake_max, 0.)

    return final_gas, final_brake
