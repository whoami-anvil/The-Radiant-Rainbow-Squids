#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 16:18:55 2021

This is the simulated Sandshark front seat


@author: BWSI AUV Challenge Instructional Staff
"""
import sys

from BWSI_Sandshark import Sandshark
from Sandshark_Interface import SandsharkServer
import threading

import time

class FrontSeat():
    # we assign the mission parameters on init
    def __init__(self, port=8000, warp=1):
        # start up the vehicle, in setpoint mode
        self.__vehicle = Sandshark(latlon=(42.3428096, -71.0901760),
                                   depth=1.0, 
                                   speed_knots=0.0,
                                   heading=70.0,
                                   rudder_position=0.0,
                                   engine_speed='STOP',
                                   engine_direction='AHEAD',
                                   datum=(42.3,-71.1))
        
        # front seat acts as server
        self.__server = SandsharkServer(port=port)
        self.__current_time = time.time()
        self.__start_time = self.__current_time
        self.__warp = warp
    
    
    def run(self):
        try:
            # start up the server
            server = threading.Thread(target=self.__server.run, args=())
            server.start()

            while True:
                now = time.time()
                delta_time = (now-self.__current_time) * self.__warp
                msg = self.__vehicle.update_state(delta_time)
                self.__server.send_command(msg)
                self.__current_time = now
                
                msgs = self.__server.receive_mail()
                if len(msgs) > 0:
                    print("\nReceived from backseat:")
                    for msg in msgs:
                        self.parse_payload_command(str(msg, 'utf-8'))
                        print(f"{str(msg, 'utf-8')}")
                
                time.sleep(1/self.__warp)
        except:
            self.__server.cleanup()
            server.join()
            
    def parse_payload_command(self, msg):
        # the only one I care about for now is BPRMB
        vals = msg.split(',')
        if vals[0] == '$BPRMB':
                
            # heading / rudder request
            if vals[2] != '':
                heading_mode = int(vals[7][:-3])
                if heading_mode == 0:
                    # this is a heading request!
                    print("SORRY, I DO NOT ACCEPT HEADING REQUESTS! I ONLY HAVE CAMERA SENSOR!")
                elif heading_mode == 1:
                    # this is a rudder adjustment!
                    rudder = float(vals[2])
                    print(f"SETTING RUDDER TO {rudder} DEGREES")
                    self.__vehicle.set_rudder(rudder)
                
            # speed request
            if vals[5] != '':
                speed_mode = int(vals[6])
                if speed_mode == 0:
                    RPM = int(vals[5])
                    print(f"SETTING THRUSTER TO {RPM} RPM")
                    self.__vehicle.set_rpm(RPM)
                elif speed_mode == 1:
                    # speed_request
                    print("SORRY, RPM SPEED REQUESTS ONLY! I HAVE NO GPS!")

def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = 8042
        
    print(f"port = {port}")
        
    front_seat = FrontSeat(port=port)
    front_seat.run()

if __name__ == '__main__':
    main()    