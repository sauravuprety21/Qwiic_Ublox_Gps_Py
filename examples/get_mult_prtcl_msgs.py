import serial
import sys
import time

import pynmea2
sys.path.append('..')
from ublox_gps.ublox_gps import UbloxGps, NMEACfg, NAV_MSGS
from ublox_gps.sparkfun_predefines import NAV_CLS

port = '/dev/serial0'
ser = serial.Serial(port, baudrate=38400, timeout=1)
gps = UbloxGps(ser)


def main():
    NMEACfg.disable_all(gps)
    NMEACfg.enable_msg(gps, 'GSA', 1)

    try:
        while 1:
            try:
                line = ser.readline().decode('utf-8')
                msg = pynmea2.parse(line)
                print(repr(msg))

                for ubx_msg in ['PVT','COV']:
                    gps.send_message(NAV_CLS, NAV_MSGS.get(ubx_msg))

                for ubx_msg in ['PVT','COV']:
                    cls_name, msg_name, payload = parse_tool.receive_from(gps.hard_port)
                    print(ubx_msg, payload)              

            except (ValueError, IOError) as err:
                print(err)
    
    finally:
        ser.close()


if __name__=="__main__": main()
