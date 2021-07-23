#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 16:18:55 2021

This is the simulated Sandshark front seat


@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import time
import threading
import time
import datetime

import utm

from Image_Processor import ImageProcessor
from AUV_Controller import AUVController

from pynmea2 import pynmea2
import BluefinMessages
from Sandshark_Interface import SandsharkClient

class BackSeat():
    # we assign the mission parameters on init
    def __init__(self, host='localhost', port=8000, warp=1):
        
        # back seat acts as client
        self.__client = SandsharkClient(host=host, port=port)
        self.__current_time = datetime.datetime.utcnow().timestamp()
        self.__start_time = self.__current_time
        self.__warp = warp
        
        self.__auv_state = dict([
            ('position', (None, None)),
            ('latlon', None),
            ('heading', None),
            ('depth', None),
            ('altitude', None),
            ('roll', None),
            ('pitch', None),
            ('last_fix_time', None)
            ])
        
        # we'll use the first navigation update as datum
        self.__datum = None
        
        # set to PICAM for the real camera
        self.__buoy_detector = ImageProcessor(camera='SIM')
        self.__autonomy = AUVController()
    
    def run(self):
        try:
            # connect the client
            client = threading.Thread(target=self.__client.run, args=())
            client.start()

            msg = BluefinMessages.BPLOG('ALL', 'ON')
            self.send_message(msg)
            
            ### These flags are for the test code. Remove them after the initial test!
            engine_started = False
            turned = False
            while True:
                now = datetime.datetime.utcnow().timestamp()
                delta_time = (now-self.__current_time) * self.__warp

                self.send_status()
                self.__current_time += delta_time
                
                msgs = self.get_mail()
                if len(msgs) > 0:
                    print("\nReceived from Frontseat:")
                    for msg in msgs:
                        print(f"{str(msg, 'utf-8')}")
                        self.process_message(str(msg, 'utf-8'))
                        # print(f"{self.__auv_state}")

                ### ---------------------------------------------------------- #
                ### Here should be the request for a photo from the camera
                ### img = self.__camera.acquire_image()
                ###
                ### Here you process the image and return the angles to target
                ### green, red = self.__detect_buoys(img)
                self.__buoy_detector.run(self.__auv_state)
                ### ---------------------------------------------------------- #
                
                
                ### self.__autonomy.decide() probably goes here!
                ### ---------------------------------------------------------- #
                
                ### turn your output message into a BPRMB request! 

                time.sleep(1/self.__warp)

                
                # ------------------------------------------------------------ #
                # ----This is example code to show commands being issued
                # ------------------------------------------------------------ #
                if True:
                    print(f"{self.__current_time - self.__start_time}")
                    if not engine_started and (self.__current_time - self.__start_time) > 3:
                        ## We want to change the speed. For now we will always use the RPM (1500 Max)
                        self.__current_time = datetime.datetime.utcnow().timestamp()
                        # This is the timestamp format from NMEA: hhmmss.ss
                        hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]

                        cmd = f"BPRMB,{hhmmss},,,,750,0,1"
                        # NMEA requires a checksum on all the characters between the $ and the *
                        # you can use the BluefinMessages.checksum() function to calculate
                        # and write it like below. The checksum goes after the *
                        msg = f"${cmd}*{hex(BluefinMessages.checksum(cmd))[2:]}"
                        self.send_message(msg)
                        engine_started = True

                    if not turned and (self.__current_time - self.__start_time) > 30:
                        ## We want to set the rudder position, use degrees plus or minus
                        ## This command is how much to /change/ the rudder position, not to 
                        ## set the rudder
                        self.__current_time = datetime.datetime.utcnow().timestamp()
                        hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]

                        cmd = f"BPRMB,{hhmmss},-15,,,750,0,1"
                        msg = f"${cmd}*{hex(BluefinMessages.checksum(cmd))[2:]}"
                        self.send_message(msg)
                        turned = True
                    
                # ------------------------------------------------------------ #
                # ----End of example code
                # ------------------------------------------------------------ #
                
                
        except:
            self.__client.cleanup()
            client.join()
          
        
    def process_message(self, msg):
        # DEAL WITH INCOMING BFNVG MESSAGES AND USE THEM TO UPDATE THE
        # STATE IN THE CONTROLLER!
        
        # JRE: skipping the checksum check for now!
                
        fields = msg.split(',')
        if fields[0] == '$BFNVG':
            # don't care about message timestamp
            #nvg_time = self.receive_nmea_time(fields[1])
                    
            # really only care about heading and position for now
            self.__auv_state['latlon'] = self.receive_nmea_latlon(fields[2],fields[3], fields[4], fields[5])
                        
            if self.__datum is None:
                # on first navigation update, set datum
                self.__datum = self.__auv_state['latlon']
                self.__datum_position = utm.from_latlon(self.__datum[0], self.__datum[1])
                self.__position = (0, 0)
            else:
                self.__auv_state['position'] = self.__get_local_position()

            self.__auv_state['datum'] = self.__datum
            self.__auv_state['altitude'] = float(fields[7])        
            self.__auv_state['depth'] = float(fields[8])
            self.__auv_state['heading'] = float(fields[9])
            self.__auv_state['roll'] = float(fields[10])
            self.__auv_state['pitch'] = float(fields[11])
            fields2 = fields[12].split('*')
            self.__auv_state['last_fix_time'] = self.receive_nmea_time(fields2[0])
            
        else:
            print(f"I do not know how to process this message type: {fields[0]}")
            
        
        
    def send_message(self, msg):
        print(f"sending message {msg}...")
        self.__client.send_message(msg)    
        
    def send_status(self):
        #print("sending status...")
        self.__current_time = time.time()
        hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]
        msg = BluefinMessages.BPSTS(hhmmss, 1, 'BWSI Autonomy OK')
        self.send_message(msg)
            
    def get_mail(self):
        msgs = self.__client.receive_mail()
        return msgs
    
    def receive_nmea_time(self, hhmmss):
        tm = datetime.datetime.utcnow()
        nvg_time = datetime.datetime(tm.year,
                                     tm.month,
                                     tm.day,
                                     int(hhmmss[0:2]), 
                                     int(hhmmss[2:4]), 
                                     int(hhmmss[4:6]),
                                     0)
        
        return nvg_time
    
    def receive_nmea_latlon(self, latdeg, lathemi, londeg, lonhemi):
        latitude = int(latdeg[0:2]) + float(latdeg[2:]) / 60
        if lathemi == 'S':
            latitude = -latitude
        
        longitude = int(londeg[0:3]) + float(londeg[3:]) / 60
        if lonhemi == 'W':
            longitude = -longitude
            
        return (latitude, longitude)
    
    def __get_local_position(self):
        # check that datum is in the same UTM zone, if not, shift datum
        local_pos = utm.from_latlon(self.__auv_state['latlon'][0],
                                    self.__auv_state['latlon'][1],
                                    force_zone_number=self.__datum_position[2],
                                    force_zone_letter=self.__datum_position[3])
        
        return (local_pos[0]-self.__datum_position[0], local_pos[1]-self.__datum_position[1])

    
    
    
            
def main():
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = "localhost"
        
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    else:
        port = 8042
    
    print(f"host = {host}, port = {port}")
    backseat = BackSeat(host=host, port=port)
    backseat.run()
    
            
if __name__ == '__main__':
    main()