import serial
import sys
import time

sys.path.append('..')

from ublox_gps.ublox_gps import UbloxGps

port = '/dev/serial0'
ser = serial.Serial(port, baudrate=38400, timeout=1)
gps = UbloxGps(ser)

def main():
    
    try:
        _try_again = True 
        while(_try_again):
            try:
                prev_cfg = gps.get_UART1_cfg()
                
                _try_again = False
            except (ValueError, IOError) as err:
                print(err)

        
        print(gps.set_baudrate_UART1(19200))
        print(ser.baudrate)
        time.sleep(1)
        _try_again = True

        while(_try_again):
            try:
                crrnt_cfg = gps.get_UART1_cfg()
            
                _try_again = False
            except(ValueError, IOError) as err:
                print(err)       
        

    finally:
        ser.close()


if __name__=="__main__": main()
