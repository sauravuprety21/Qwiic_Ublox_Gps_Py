import serial
import sys
import traceback

sys.path.append('..')
from ublox_gps.ublox_gps import UbloxGps


port = '/dev/serial0'
ser = serial.Serial(port, baudrate=9600, timeout=1)
gps = UbloxGps(ser)

def main():
    try:
        for i in range(3):
            try:
                msg = gps.get_UART1_cfg()
                print(msg)
            except (ValueError, IOError) as err:
                print('-'*60)
                traceback.print_exc(file=sys.stdout)
                print('-'*60)

    finally:
        ser.close()


if __name__=="__main__": main()

