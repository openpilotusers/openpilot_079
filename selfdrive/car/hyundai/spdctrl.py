import math
import numpy as np
from common.numpy_fast import clip, interp

from selfdrive.car.hyundai.spdcontroller  import SpdController

import common.log as trace1


class Spdctrl(SpdController):
    def __init__(self, CP=None):
        super().__init__( CP )
        self.cv_Raio = 0.4
        self.cv_Dist = -5
        self.steer_mode = ""
        self.cruise_gap = 0.0

    def update_lead(self, CS,  dRel, yRel, vRel):
        lead_set_speed = int(round(self.cruise_set_speed_kph))
        lead_wait_cmd = 600

        #dRel, yRel, vRel = self.get_lead( sm, CS )
        if CS.lead_distance < 150:
            dRel = CS.lead_distance # 레이더 인식거리 학인되면 인식거리를 dRel(이온 차간간격)으로 치환
            vRel = CS.lead_objspd

        dst_lead_distance = (CS.clu_Vanz*self.cv_Raio)   # 기준 유지 거리
        
        if dst_lead_distance > 100:
            dst_lead_distance = 100
        #elif dst_lead_distance < 15:
            #dst_lead_distance = 15

        if dRel < 150: #앞차와의 간격이 150미터 미만이면, 즉 앞차가 인식되면,
            self.time_no_lean = 0
            d_delta = dRel - float(dst_lead_distance)  # d_delta = 앞차간격(이온값) - 유지거리
            lead_objspd = vRel  # 선행차량 상대속도.
        else:
            d_delta = 0
            lead_objspd = 0
 
        if CS.driverAcc_time: #운전자가 가속페달 밟으면 크루즈 설정속도를 현재속도로 동기화
            lead_set_speed = int(round(CS.clu_Vanz))
            self.seq_step_debug = "운전자가속"
            lead_wait_cmd = 15
        # 거리 유지 조건
        elif d_delta < 0: # 기준유지거리(현재속도*0.4)보다 가까이 있게 된 상황 
            if lead_objspd < -30 or (dRel < 60 and CS.clu_Vanz > 60 and lead_objspd < -5): # 끼어든 차가 급감속 하는 경우
                self.seq_step_debug = "기준내,-3"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -3)
            elif lead_objspd < -20 or (dRel < 80 and CS.clu_Vanz > 80 and lead_objspd < -5):  # 끼어든 차가 급감속 하는 경우
                self.seq_step_debug = "기준내,-2"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -3)
            elif lead_objspd < -10:
                self.seq_step_debug = "기준내,-1"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 30, -2)
            elif lead_objspd < 0:
                self.seq_step_debug = "기준내,-1"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -2)
            elif lead_objspd >= 0 and CS.VSetDis > (CS.clu_Vanz + 10) and int(CS.clu_Vanz * 0.3) < dRel: # 끼어든 차가 가속중 크루즈 설정속도가 현재 차속+15 보다 큰 상황
                self.seq_step_debug = "기준내,-1"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 100, -1)
            else:
                self.seq_step_debug = "d<0,거리유지"

        # 선행차량이 멀리 있는 상태에서 감속 조건
        elif lead_objspd < -25 and dRel < 100:  #차간거리 100이하 상대속도 -30 미만
            self.seq_step_debug = "s<-25,d<100,-2"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 20, -3)
        elif lead_objspd < -20 and dRel < 80:
            self.seq_step_debug = "s<-20,d<80,-2"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 30, -3)
        elif lead_objspd < -15 and dRel < 60:
            self.seq_step_debug = "s<-15,d<60,-2"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 40, -3)
        elif lead_objspd < -10 and dRel < 50:
            self.seq_step_debug = "s<-10,d<50,-2"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -2)
        elif lead_objspd < -5 and dRel < 40:
            self.seq_step_debug = "s<-5,d<40,-2"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 60, -2)
        elif self.cruise_set_speed_kph > (int(round((CS.clu_Vanz))) + 1):  #이온설정속도가 차량속도보다 큰경우
            if lead_objspd > 5 and CS.clu_Vanz > 25 and CS.VSetDis < 40: # 처음출발시 선행차량 급가속할 때 차량속도 20되기 전 최대한 설정속도 업 한 후 대기
                self.seq_step_debug = "SS>VS,초가"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 5, 5)
            elif lead_objspd > 5 and CS.clu_Vanz > 40 and CS.VSetDis < 55: # 처음출발시 선행차량 급가속할 때 차량속도 20되기 전 최대한 설정속도 업 한 후 대기
                self.seq_step_debug = "SS>VS,중가"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 5, 5)
            elif lead_objspd > 5 and CS.clu_Vanz > 60 and CS.VSetDis < 75: # 처음출발시 선행차량 급가속할 때 차량속도 20되기 전 최대한 설정속도 업 한 후 대기
                self.seq_step_debug = "SS>VS,종가"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 5, 5)
            elif lead_objspd >= 0 and CS.clu_Vanz >= (int(CS.VSetDis) - 6) and int(CS.clu_Vanz * 0.5) < dRel:
                self.seq_step_debug = "SS>VS,+1"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 55, 3)
            elif CS.clu_Vanz > 50 and lead_objspd < 0 and int(CS.clu_Vanz * 0.3) > dRel:
                self.seq_step_debug = "SS>VS,-1"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 100, -1)
            elif CS.clu_Vanz < 30 and lead_objspd < 0 and CS.VSetDis > 30:
                self.seq_step_debug = "SS>VS,30이하"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 20, -3)
            else:
                self.seq_step_debug = "SS>VS,거리유지"
        elif lead_objspd >= 0 and CS.clu_Vanz >= (int(CS.VSetDis) - 6) and int(CS.clu_Vanz * 0.5) < dRel:
            self.seq_step_debug = "일반가속"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 55, 3)
        elif lead_objspd < 0 and int(CS.clu_Vanz * 0.3) > dRel:
            self.seq_step_debug = "일반감속"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 100, -1)
        else:
            self.seq_step_debug = "거리유지"

        return lead_wait_cmd, lead_set_speed

    def update_curv(self, CS, sm, model_speed):
        wait_time_cmd = 0
        set_speed = self.cruise_set_speed_kph

        # 2. 커브 감속.
        #if self.cruise_set_speed_kph >= 100:
        if CS.out.cruiseState.modeSel == 1:
            if model_speed < 60 and CS.clu_Vanz > 40:
                set_speed = self.cruise_set_speed_kph - int(CS.clu_Vanz * 0.2)
                self.seq_step_debug = "커브감속-4"
                wait_time_cmd = 100
            elif model_speed < 70 and CS.clu_Vanz > 40:
                set_speed = self.cruise_set_speed_kph - int(CS.clu_Vanz * 0.15)
                self.seq_step_debug = "커브감속-3"
                wait_time_cmd = 100
            elif model_speed < 80 and CS.clu_Vanz > 40:
                set_speed = self.cruise_set_speed_kph - int(CS.clu_Vanz * 0.1)
                self.seq_step_debug = "커브감속-2"
                wait_time_cmd = 100
            elif model_speed < 90 and CS.clu_Vanz > 40:
                set_speed = self.cruise_set_speed_kph - int(CS.clu_Vanz * 0.05)
                self.seq_step_debug = "커브감속-1"
                wait_time_cmd = 100

        return wait_time_cmd, set_speed


    def update_log(self, CS, set_speed, target_set_speed, long_wait_cmd ):
        if CS.out.cruiseState.modeSel == 0:
            self.steer_mode = "오파모드"
        elif CS.out.cruiseState.modeSel == 1:
            self.steer_mode = "차간+커브"
        elif CS.out.cruiseState.modeSel == 2:
            self.steer_mode = "차간ONLY"
        elif CS.out.cruiseState.modeSel == 3:
            self.steer_mode = "편도1차선"

        if self.cruise_gap != CS.cruiseGapSet:
            self.cruise_gap = CS.cruiseGapSet

        str3 = '주행모드={:s}  설정속도={:03.0f}/{:03.0f}  타이머={:03.0f}/{:03.0f}'.format( self.steer_mode, set_speed, CS.VSetDis, long_wait_cmd, self.long_curv_timer )
        str4 = '  차=D:{:03.0f}/V:{:03.0f}  CG={:1.0f}  구분={:s}'.format(  CS.lead_distance, CS.lead_objspd, self.cruise_gap, self.seq_step_debug )

        str5 = str3 + str4
        trace1.printf2( str5 )
