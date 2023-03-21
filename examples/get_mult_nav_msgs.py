import serial

import sys
sys.path.append("..")

from ublox_gps.ublox_gps import *
from ublox_gps.core import *
from ublox_gps.sparkfun_predefines import NAV_CLS

PORT = '/dev/serial0'

ser = serial.Serial(PORT, baudrate=9600, timeout=1)
gps = UbloxGps(ser)



def main():

    try:
        while(1):
            try:
                gps.send_message(NAV_CLS, NAV_MSGS.get('PVT'))
                gps.send_message(NAV_CLS, NAV_MSGS.get('COV'))
                parse_tool = Parser([NAV_CLS])
                cls_name, msg_name, payload = parse_tool.receive_from(gps.hard_port)
                print(msg_name, payload.iTOW)                             
                
            except (ValueError, IOError) as err:
                print(err)
    
    finally:
        ser.close()



if __name__ == "__main__": main()

