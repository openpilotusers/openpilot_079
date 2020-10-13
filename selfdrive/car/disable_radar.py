#!/usr/bin/env python3
import traceback

import cereal.messaging as messaging
import panda
from panda import Panda
from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from selfdrive.swaglog import cloudlog

RADAR_ADDR = 0x7D0
EXT_DIAG_REQUEST = b'\x10\x85'
EXT_DIAG_RESPONSE = b''
COM_CONT_REQUEST = b''
COM_CONT_RESPONSE = b'\x50\x85\xAA\xAA\xAA\xAA\xAA'

def disable_radar(logcan, sendcan, bus, timeout=1, retry=5, debug=False):
  print(f"radar disable {hex(RADAR_ADDR)} ...")
  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  for i in range(retry):
    try:
      # enter extended diagnostic session
      query = IsoTpParallelQuery(sendcan, logcan, bus, [RADAR_ADDR], [EXT_DIAG_REQUEST], [EXT_DIAG_RESPONSE], debug=debug)
      for addr, dat in query.get_data(timeout).items():
        print("radar communication control disable tx/rx ...")
        # communication control disable tx and rx
        query = IsoTpParallelQuery(sendcan, logcan, bus, [RADAR_ADDR], [COM_CONT_REQUEST], [COM_CONT_RESPONSE], debug=debug)
        query.get_data(0)
        return True
      print(f"radar disable retry ({i+1}) ...")
    except Exception:
      cloudlog.warning(f"radar disable exception: {traceback.format_exc()}")

  return False


if __name__ == "__main__":
  import time
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)
  disabled = disable_radar(logcan, sendcan, 0, debug=False)
  print(f"disabled: {disabled}")
