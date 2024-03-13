import json
#from multiprocessing import parent_process
from multiprocessing.connection import Client, Listener
from re import A
from typing import Dict
import requests
import time
from datetime import datetime, timedelta
from datetime import datetime
from dronekit import Vehicle, connect, VehicleMode
import time
#--- Start the Software In The Loop (SITL)
import requests
from dronekit import connect, VehicleMode
import time
from dronekit import connect
import random
from requests.api import request
import cv2
import datetime

a = 0

class Communication:
    f = open("log.txt", "a")
    def __init__(self):
        
        self.f.write("----This is the Start of the log----")
    
    
    def closing(self):
        self.f.close()

    def drawHud(self,width,height):
        xRight = int(width/4)
        yRight = int(height/10)
        xLeft = xRight*3
        yLeft = yRight*9
        
        return xRight, yRight, xLeft, yLeft

    def __init__(self):
        self.teamnumber = 199
        self.url_value_given_by_teknofest = "http://10.0.0.15:64559" #2022 savasan
        #self.url_value_given_by_teknofest = "http://192.168.20.10:64559" --> probably the teknofest address
        #self.url_value_given_by_teknofest = "http://127.0.0.1:8000"
        
    
    def connectDrone(self):
        print(">>>> Connecting with the UAV <<<")
        """sitl = dronekit_sitl.start_default()   #(sitl.start)
        connection_string = sitl.connection_string()
        connection_string = "/dev/ttyUSB1"#/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10JXPUC-if00-port0"#/dev/ttyUSB0"
        print(">>>> Connecting with the UAV <<<")"""
        #connection_string = "/dev/ttyUSB1"#/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10JXPUC-if00-port0"#/dev/ttyUSB0"
        #sitl = dronekit_sitl.start_default()
        #connection_string = sitl.connection_string()
        #vehicle = connect(connection_string, wait_ready=True, baud=57600)
        #vehicle = connect("127.0.0.1:" + str(14560),baud=57600)
        
        vehicle = connect("127.0.0.1:" + str(14570))     #- wait_ready flag hold the program untill all the parameters are been read (=, not .) Ucaga buradan baglanir
        return vehicle

    def connectToServer(self,s):
        team_payload ={
            "kadi": "sufaiprojeekibi",
            "sifre": "61aynprs23"
         }

        r = s.post(self.url_value_given_by_teknofest + '/api/giris', json = team_payload)
        #print("Connection Situation(must be <<200>>): "+str(r)+"-----------"+r.text)
        return s

    def sendKamikaze(self, passVal, utc, qr_text):
        rand = random.randint(0, 979)
        rand2 = random.randint(2, 4)
        rand_bitis = random.randint(0, 19)
        #rand2_bitis = random.randint(2, 4)
        miliS = utc[3]
        second = utc[2]
        minute = utc[1]
        hour = utc[0]
        """if second < 0:
            minute-1
        if minute <= 0:
            hour -=1
"""
        milis_bitis = rand_bitis
        second_bitis = utc[2]
        minute_bitis = utc[1]
        hour_bitis = utc[0]
        if second_bitis < 0:
            minute_bitis-1
        if minute_bitis <= 0:
            hour_bitis -=1
        print(second)
        print(miliS)
        a = {
        "kamikazeBaslangicZamani": {
        "saat": hour,
        "dakika": minute,
        "saniye": second,
        "milisaniye": miliS - 98
        },
        "kamikazeBitisZamani": {
        "saat": hour_bitis,
        "dakika": minute_bitis,
        "saniye": second_bitis,
        "milisaniye": miliS
        },
        "qrMetni": qr_text
        }
        print("--------------------------QR TEXT------------------")
        print(qr_text)
        print("""\n

     QQQQQQQQQ     RRRRRRRRRRRRRRRRR        DDDDDDDDDDDDD      EEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTTTTTTTTTTTTEEEEEEEEEEEEEEEEEEEEEE       CCCCCCCCCCCCCTTTTTTTTTTTTTTTTTTTTTTT
   QQ:::::::::QQ   R::::::::::::::::R       D::::::::::::DDD   E::::::::::::::::::::ET:::::::::::::::::::::TE::::::::::::::::::::E    CCC::::::::::::CT:::::::::::::::::::::T
 QQ:::::::::::::QQ R::::::RRRRRR:::::R      D:::::::::::::::DD E::::::::::::::::::::ET:::::::::::::::::::::TE::::::::::::::::::::E  CC:::::::::::::::CT:::::::::::::::::::::T
Q:::::::QQQ:::::::QRR:::::R     R:::::R     DDD:::::DDDDD:::::DEE::::::EEEEEEEEE::::ET:::::TT:::::::TT:::::TEE::::::EEEEEEEEE::::E C:::::CCCCCCCC::::CT:::::TT:::::::TT:::::T
Q::::::O   Q::::::Q  R::::R     R:::::R       D:::::D    D:::::D E:::::E       EEEEEETTTTTT  T:::::T  TTTTTT  E:::::E       EEEEEEC:::::C       CCCCCCTTTTTT  T:::::T  TTTTTT
Q:::::O     Q:::::Q  R::::R     R:::::R       D:::::D     D:::::DE:::::E                     T:::::T          E:::::E            C:::::C                      T:::::T        
Q:::::O     Q:::::Q  R::::RRRRRR:::::R        D:::::D     D:::::DE::::::EEEEEEEEEE           T:::::T          E::::::EEEEEEEEEE  C:::::C                      T:::::T        
Q:::::O     Q:::::Q  R:::::::::::::RR         D:::::D     D:::::DE:::::::::::::::E           T:::::T          E:::::::::::::::E  C:::::C                      T:::::T        
Q:::::O     Q:::::Q  R::::RRRRRR:::::R        D:::::D     D:::::DE:::::::::::::::E           T:::::T          E:::::::::::::::E  C:::::C                      T:::::T        
Q:::::O     Q:::::Q  R::::R     R:::::R       D:::::D     D:::::DE::::::EEEEEEEEEE           T:::::T          E::::::EEEEEEEEEE  C:::::C                      T:::::T        
Q:::::O  QQQQ:::::Q  R::::R     R:::::R       D:::::D     D:::::DE:::::E                     T:::::T          E:::::E            C:::::C                      T:::::T        
Q::::::O Q::::::::Q  R::::R     R:::::R       D:::::D    D:::::D E:::::E       EEEEEE        T:::::T          E:::::E       EEEEEEC:::::C       CCCCCC        T:::::T        
Q:::::::QQ::::::::QRR:::::R     R:::::R     DDD:::::DDDDD:::::DEE::::::EEEEEEEE:::::E      TT:::::::TT      EE::::::EEEEEEEE:::::E C:::::CCCCCCCC::::C      TT:::::::TT      
 QQ::::::::::::::Q R::::::R     R:::::R     D:::::::::::::::DD E::::::::::::::::::::E      T:::::::::T      E::::::::::::::::::::E  CC:::::::::::::::C      T:::::::::T      
   QQ:::::::::::Q  R::::::R     R:::::R     D::::::::::::DDD   E::::::::::::::::::::E      T:::::::::T      E::::::::::::::::::::E    CCC::::::::::::C      T:::::::::T      
     QQQQQQQQ::::QQRRRRRRRR     RRRRRRR     DDDDDDDDDDDDD      EEEEEEEEEEEEEEEEEEEEEE      TTTTTTTTTTT      EEEEEEEEEEEEEEEEEEEEEE       CCCCCCCCCCCCC      TTTTTTTTTTT      
             Q:::::Q                                                                                                                                                         
              QQQQQQ                                                               
                                                        
                                                        
                                                        
                                                        
                                                        
                                                        
                                                        """)
        
        r = passVal.post(self.url_value_given_by_teknofest + '/api/kamikaze_bilgisi', json = a)
        return r
                
    def sendLockInfo(self, passVal, TrackingTimerEnd, TrackingTimerStart):
        #print("I am in send lock info")
        
        kitlenme_payload = {
        "kilitlenmeBaslangicZamani": {
        "saat":TrackingTimerStart[0],
        "dakika":TrackingTimerStart[1],
        "saniye": TrackingTimerStart[2],
        "milisaniye": TrackingTimerStart[3]
        },
        "kilitlenmeBitisZamani": {
        "saat": TrackingTimerEnd[0],
        "dakika": TrackingTimerEnd[1],
        "saniye": TrackingTimerEnd[2],
        "milisaniye": TrackingTimerEnd[3]
        },
        "otonom_kilitlenme": 0
        }
        r = passVal.post(self.url_value_given_by_teknofest+'/api/kilitlenme_bilgisi', json = kitlenme_payload)
        self.f.write(str(kitlenme_payload))
        #print(kitlenme_payload)
        result = json.dumps(kitlenme_payload)
        #print("hey you are in the loop")
        print("""\n ██╗  ██╗██╗██╗     ██╗         ███╗   ███╗ █████╗ ██████╗ ██╗  ██╗
                    ██║ ██╔╝██║██║     ██║         ████╗ ████║██╔══██╗██╔══██╗██║ ██╔╝
                    █████╔╝ ██║██║     ██║         ██╔████╔██║███████║██████╔╝█████╔╝ 
                    ██╔═██╗ ██║██║     ██║         ██║╚██╔╝██║██╔══██║██╔══██╗██╔═██╗ 
                    ██║  ██╗██║███████╗███████╗    ██║ ╚═╝ ██║██║  ██║██║  ██║██║  ██╗
                    ╚═╝  ╚═╝╚═╝╚══════╝╚══════╝    ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝
                    ███████╗██╗   ██╗███████╗ █████╗ ██╗    
                    ██╔════╝██║   ██║██╔════╝██╔══██╗██║    
                    ███████╗██║   ██║█████╗  ███████║██║    
                    ╚════██║██║   ██║██╔══╝  ██╔══██║██║     
                    ███████║╚██████╔╝██║     ██║  ██║██║   
                    ╚══════╝ ╚═════╝ ╚═╝     ╚═╝  ╚═╝╚═╝    
                                                                
                    ██╗  ██╗██╗██╗     ██╗         ███╗   ███╗ █████╗ ██████╗ ██╗  ██╗
                    ██║ ██╔╝██║██║     ██║         ████╗ ████║██╔══██╗██╔══██╗██║ ██╔╝
                    █████╔╝ ██║██║     ██║         ██╔████╔██║███████║██████╔╝█████╔╝ 
                    ██╔═██╗ ██║██║     ██║         ██║╚██╔╝██║██╔══██║██╔══██╗██╔═██╗ 
                    ██║  ██╗██║███████╗███████╗    ██║ ╚═╝ ██║██║  ██║██║  ██║██║  ██╗
                    ╚═╝  ╚═╝╚═╝╚══════╝╚══════╝    ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝  

                    ███████╗██╗   ██╗███████╗ █████╗ ██╗   
                    ██╔════╝██║   ██║██╔════╝██╔══██╗██║    
                    ███████╗██║   ██║█████╗  ███████║██║     
                    ╚════██║██║   ██║██╔══╝  ██╔══██║██║     
                    ███████║╚██████╔╝██║     ██║  ██║██║    
                    ╚══════╝ ╚═════╝ ╚═╝     ╚═╝  ╚═╝╚═╝    
                                                                
                    ██╗  ██╗██╗██╗     ██╗         ███╗   ███╗ █████╗ ██████╗ ██╗  ██╗
                    ██║ ██╔╝██║██║     ██║         ████╗ ████║██╔══██╗██╔══██╗██║ ██╔╝
                    █████╔╝ ██║██║     ██║         ██╔████╔██║███████║██████╔╝█████╔╝ 
                    ██╔═██╗ ██║██║     ██║         ██║╚██╔╝██║██╔══██║██╔══██╗██╔═██╗ 
                    ██║  ██╗██║███████╗███████╗    ██║ ╚═╝ ██║██║  ██║██║  ██║██║  ██╗
                    ╚═╝  ╚═╝╚═╝╚══════╝╚══════╝    ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝                                                                
                                                        
                                                        
                                                        
                                                        
                                                        
                                                        
                                                        """)

        return r
        
    
    def getTimeFromServer(self):
        r = requests.get(self.url_value_given_by_teknofest + '/api/sunucusaati')
        parsed_data = r.json()
        #print(parsed_data)

        #print(type(parsed_data))
        #print(parsed_data)
        hours_value = parsed_data["saat"]
        minutes_values = parsed_data["dakika"]
        seconds_value = parsed_data["saniye"]
        milisecs_value = parsed_data["milisaniye"]
        #print("time respose")
        #print(r)
        return int(hours_value), int(minutes_values), int(seconds_value), int(milisecs_value)

   
    def getCoordinates(self,passVal):
        r = passVal.get(self.url_value_given_by_teknofest + '/api/qr_koordinati')
        parsed_data = r.json()
        print(parsed_data)

        #enlem = parsed_data["qrEnlem"]
        #boylam = parsed_data["qrBoylam"]
        #print("--------------------KOORDINAT VERSI -----------------------")
        #print(parsed_data)
        #return enlem, boylam
        return parsed_data
   
    def sendTelemetry(self,passVal,vehicle, holder_for_lock, utc, msg):
        global a
        battery_percent = (((vehicle.battery.voltage - 14) * 100) / 2.8)
        code = cv2.waitKey(1)
        if(vehicle.mode=="AUTO" or code == ord('o')):
            a = 1
        elif code == ord("p"):
            a=0
        x = datetime.datetime.now()
        telemetry_payload={
        "takim_numarasi": 19,        # Yarışma Sırasında Güncelle!!!
        "IHA_enlem": vehicle.location.global_frame.lat,
        "IHA_boylam": vehicle.location.global_frame.lon,
        "IHA_irtifa": (vehicle.location.global_frame.alt),
        "IHA_dikilme": int(vehicle.attitude.pitch*57.2958),
        "IHA_yonelme": int(vehicle.attitude.yaw*57.2958),
        "IHA_yatis": int(vehicle.attitude.roll*57.2958),
        "IHA_hiz": int(vehicle.velocity[0] ) ,
        "IHA_batarya": int(battery_percent),
        "IHA_otonom": a,   
        "IHA_kilitlenme": holder_for_lock,
        "Hedef_merkez_X": int(msg[1]),
        "Hedef_merkez_Y":int(msg[2]),
        "Hedef_genislik": int(msg[4]),
        "Hedef_yukseklik": int(msg[3]),
        "GPSSaati": {
        "saat": x.hour,
        "dakika": x.minute,
        "saniye": x.second,
        "milisaniye": int(round(int(x.microsecond)/1000,0))
        }
        }

        print(telemetry_payload)
        r = passVal.post(self.url_value_given_by_teknofest+'/api/telemetri_gonder', json = telemetry_payload)
        result = json.dumps(telemetry_payload)
        #print("'sendTelemetry' is called")
        return r


    def Run(self,passVal,vehicle,msg,hours_value,minutes_values,seconds_value,
    milisecs_value, hours_current,minutes_current,seconds_current, milisecs_current,continuity_of_lock, flag_for_start_time, xxxxState):

        #continuity_of_lock, flag_for_start_time,hours_value, minutes_values, seconds_value, milisecs_value = self.sendLockInfo(passVal,msg,continuity_of_lock,
        #flag_for_start_time,hours_value,minutes_values,seconds_value,milisecs_value,
        #hours_current,minutes_current,seconds_current,milisecs_current)         
        
        a1,a2,a3,a4 = self.getTimeFromServer()
        utc= [a1,a2,a3,a4]
        xflag =0
        if(xxxxState=="Tracking"):
            xflag=1
        returnedTelemetry = self.sendTelemetry(passVal,vehicle, xflag , utc, msg)
        #print("-------------------JSON SERVER RETURN AFTER SENT TELEMETRY--------------")
        print(returnedTelemetry.json())

        return  returnedTelemetry,continuity_of_lock, flag_for_start_time,hours_value, minutes_values, seconds_value, milisecs_value
        

    
