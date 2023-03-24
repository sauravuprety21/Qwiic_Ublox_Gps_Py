import pynmea2

import sys

sys.path.append('..')

from ublox_gps.ublox_gps import UbloxGps, NMEACfg

import serial

port = '/dev/serial0'
ser = serial.Serial(port, baudrate=38400, timeout=1)
gps = UbloxGps(ser)


def main():
    NMEACfg.disable_all(gps)
    NMEACfg.enable_msg(gps, 'GST', 1)
    NMEACfg.enable_msg(gps, 'GSA', 1)
    try:
        while 1:
            try:
                line = ser.readline().decode('utf-8')
                msg = pynmea2.parse(line)
                print(repr(msg))
            except (ValueError, IOError) as err:
                print(err)
    finally:
        ser.close()

if __name__=="__main__": main()
