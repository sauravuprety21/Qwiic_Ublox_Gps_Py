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
                gps.send_message(NAV_CLS, NAV_MSGS.get('SAT'))
                parse_tool = Parser([NAV_CLS])
                cls_name, msg_name, payload = parse_tool.receive_from(gps.hard_port)
                if msg_name == 'PVT':
                    print(f"Time:{payload.iTOW}\tLat:{payload.lat}\tLong:{payload.lon}\thAvv:{payload.hAcc}")
                elif msg_name =='COV':
                    print(f"Time:{payload.iTOW}\tvelNcov:{payload.velCovNN}\tposNcov:{payload.posCovNN}\n")
                else:
                    print(f"Time:{payload.iTOW}\tSIV:{payload.numSvs}")
            except (ValueError, IOError) as err:
                print(err)
    
    finally:
        ser.close()



if __name__ == "__main__": main()

