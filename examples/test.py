import serial

import sys
sys.path.append("..")

from ublox_gps.ublox_gps import UbloxGps

PORT = '/dev/serial0'

ser = serial.Serial(PORT, baudrate=9600, timeout=1)
gps = UbloxGps(ser)



def main():

    try:
        while(1):
            try:
                geo = gps.geo_coords()
                print(f"Longitude:{geo.lon}\tLatitude:{geo.lat}")

            except (ValueError, IOError) as err:
                print(err)
    
    finally:
        ser.close()



if __name__ == "__main__": main()

