# Python library for the SparkFun's line of u-Blox GPS units.
#
# SparkFun GPS-RTK NEO-M8P:
#   https://www.sparkfun.com/products/15005
# SparkFun GPS-RTK2 ZED-F9P:
#   https://www.sparkfun.com/products/15136
# SparkFun GPS ZOE-M8Q:
#   https://www.sparkfun.com/products/15193
# SparkFun GPS SAM-M8Q:
#   https://www.sparkfun.com/products/15210
# SparkFun GPS-RTK Dead Reckoning Phat ZED-F9R:
#   https://www.sparkfun.com/products/16475
# SparkFun GPS-RTK Dead Reckoning ZED-F9R:  
#   https://www.sparkfun.com/products/16344
# SparkFun GPS Dead Reckoning NEO-M9N:
#   https://www.sparkfun.com/products/15712
#
#------------------------------------------------------------------------
# Written by SparkFun Electronics, July 2020
#
# Do you like this library? Help suphard_port SparkFun. Buy a board!
#==================================================================================
# GNU GPL License 3.0
# Copyright (c) 2020 SparkFun Electronics
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ==================================================================================
#
# The following code is a dependent on work by daymople and the awesome parsing
# capabilities of ubxtranslator: https://github.com/dalymople/ubxtranslator.
#
# pylint: disable=line-too-long, bad-whitespace, invalid-name, too-many-public-methods
#

import struct
import serial


from . import sparkfun_predefines as sp
from . import core

_DEFAULT_NAME = "Qwiic GPS"
_AVAILABLE_I2C_ADDRESS = [0x42]

# Comms Port ID @COMMSPORTIDS
UART_PORT_ID = 0x01

# Protocol Masks @OUTPROTOCOL_MASKS
UBX_OUTPROTO_MASK = 0x01
NMEA_OUTPROTO_MASK = 0x01 << 1
OUTPROTO_MASKS = {"UBX": UBX_OUTPROTO_MASK, "NMEA": NMEA_OUTPROTO_MASK}


#  Ublox Messages w/i each class (non-inclusive) @UBX_MSG_VALUES
ACK_MSGS= { 'ACK':0x01, 'NAK':0x00 }

CFG_MSGS= { 'OTP':0x41,    'PIO':0x2c,      'PRT':0x00,
            'PT2':0x59,    'RST':0x04,      'SPT':0x64,
            'USBTEST':0x58,  'VALDEL':0x8c,  'VALGET':0x8b,
            'VALSET':0x8a, 'MSG': 0x01 }

ESF_MSGS= {'ALG':0x14,       'INS':0x15,    'MEAS':0x02,
              'RAW':0x03,       'RESETALG':0x13,  'STATUS':0x10}

INF_MSGS= {'DEBUG':0x04,  'ERROR':0x00,   'NOTICE':0x02,
            'TEST':0x03,   'WARNING':0x01}

MGA_MSGS= {'ACK':0x60,       'BDS_EPH':0x03,
        'BDS_ALM':0x03,   'BDS_HEALTH':0x03,      'BDS_UTC':0x03,
        'DBD_POLL':0x80,  'DBD_IO':0x80,          'GAL_EPH':0x02,
        'GAL_ALM':0x02,   'GAL_TIMEOFFSET':0x02,  'GAL_UTC':0x02}

MON_MSGS= {'COMMS':0x36,  'GNSS':0x28,  'HW3':0x37,  'PATCH':0x27,
        'PIO':0x24,    'PT2':0x2b,   'RF':0x38,   'RXR':0x21,
        'SPT':0x2f}

NAV_MSGS= {'ATT':0x05,        'CLOCK':0x22,     'COV':0x36,
        'DOP':0x04,        'EELL':0x3d,      'EOE':0x61,        'GEOFENCE':0x39,
        'HPPOSECEF':0x13,  'HPPOSLLH':0x14,  'ORB':0x34,        'POSECEF':0x01,
        'POSLLH':0x02,     'PVT':0x07,       'RELPOSNED':0x3c,  'SAT':0x35,
        'SBAS':0x32,       'SIG':0x43,       'STATUS':0x03,     'TIMBDS':0x24,
        'TIMEGAL':0x25,    'TIMEGLO':0x23,   'TIMEGPS':0x20,    'TIMELS':0x25,
        'TIMEQZSS':0x27,   'TIMEUTC':0x21,   'VELECEF':0x11,    'VELNED':0x12}

TIME_MSGS= {'TM2':0x03, 'TP':0x01, 'VRFY':0x06}


class UbloxGps(object):
    """
    UbloxGps

    Initialize the library with the given port.

    :param hard_port:   The port to use to communicate with the module, this
                        can be a serial or SPI port. If no port is given, then the library
                        assumes serial at a 38400 baud rate.

    :return:            The UbloxGps object.
    :rtype:             Object
    """

    device_name = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    def __init__(self, hard_port = None):

        if hard_port is None:
            self.hard_port = serial.Serial("/dev/serial0/", 38400, timeout=1)
        else:
            self.hard_port = hard_port


    def send_message(self, ubx_class, ubx_id, ubx_payload = None):
        """
        Sends a ublox message to the ublox module.

        :param ubx_class:   The ublox class with which to send or receive the
                            message to/from.
        :param ubx_id:      The message id under the ublox class with which
                            to send or receive the message to/from.
        :param ubx_payload: The payload to send to the class/id specified. If
                            none is given than a "poll request" is
                            initiated.
        :return: True on completion
        :rtype: boolean
        """

        SYNC_CHAR1 = 0xB5
        SYNC_CHAR2 = 0x62

        if ubx_payload == b'\x00' or ubx_payload is None:
            payload_length = 0
        elif type(ubx_payload) is not bytes:
            ubx_payload = bytes([ubx_payload])
            payload_length = len(ubx_payload)
        else:
            payload_length = len(ubx_payload)

        if payload_length > 0:
            message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2,
                                  ubx_class.id_, ubx_id, (payload_length & 0xFF),
                                  (payload_length >> 8)) + ubx_payload

        else:
            message = struct.pack('BBBBBB', SYNC_CHAR1, SYNC_CHAR2,
                                  ubx_class.id_, ubx_id, (payload_length & 0xFF),
                                  (payload_length >> 8))

        checksum = core.Parser._generate_fletcher_checksum(message[2:])

        self.hard_port.write(message + checksum)

        return True




    def enable_UART1(self, enable):
        if enable is True:
            self.send_message(sp.CFG_CLS, CFG_MSGS.get('RST'), 0x00)

        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        msg = parse_tool.receive_from(self.hard_port)
        return msg



    def get_UART1_cfg(self):
          
        self.send_message(sp.CFG_CLS, 
                          CFG_MSGS.get('PRT'), 
                          Payload.serialize(sp.CFG_CLS_PRT_POLL_MSG, [UART_PORT_ID]))
        
        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        cls_name, msg_name, cfg_payload = parse_tool.receive_from(self.hard_port)
        ack_ = parse_tool.receive_from(self.hard_port)

        return cfg_payload
    

    def set_baudrate_UART1(self, br: int):
        
        prt_cfg = self.get_UART1_cfg()
        prt_cfg = prt_cfg._replace(baudRate=br)
        
        self.send_message(sp.CFG_CLS, CFG_MSGS.get('PRT'), 
                Payload.serialize_Parsed_MSG(sp.CFG_CLS_PRT_UART_MSG,prt_cfg))
        
        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])

        ack_ = parse_tool.receive_from(self.hard_port)
    
        if ack_.name == 'ACK':
            self.hard_port.setBaudrate(br)
            return True

        return False
     


    def geo_coords(self):
        """
        Sends a poll request for the NAV class with the PVT Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and PVT Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, NAV_MSGS.get('PVT'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload
    

    def get_cov_mat(self):
        """
        Sends a poll request for the NAV class with the COV Message ID and
        parses ublox messages for the response. The payload is extracted from
        the response which is then passed to the user.

        :return: The payload of the NAV Class and COV Message ID
        :rtype: namedtuple
        """
        self.send_message(sp.NAV_CLS, NAV_MSGS.get('COV'))
        parse_tool = core.Parser([sp.NAV_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(self.hard_port)
        return payload



    def stream_nmea(self):
        """
        Reads directly from the module's data stream, by default this is NMEA
        data.

        :return: Returns NMEA data.
        :rtype: string
        """
        return self.hard_port.readline().decode('utf-8')
    


class Payload:
    
    @staticmethod
    def serialize(coreObj, payload:list):
        """
        :param coreObj:     an instance of a 'core' class, which hasattr '.fmt'
        :payload:           data memebers of a named tuple instance which is described by the coreObj

        :return: payload packed as per coreObj.fmt description
        :rtype: byte-string
        """
        return struct.pack(coreObj.fmt, *payload)
    

    @staticmethod
    def is_NamedTuple(payload_member) -> bool:
        """
        Loose check for 'namedtuple' instances, the data from these must be extracted
        """
        return isinstance(payload_member, tuple)
    


    @staticmethod
    def serialize_Parsed_MSG(msg:core.Message, _nt_payload):
        """
        Messages (core.Message)  parsed from receiver have bitfields where values are stored in flags.
        These bitfields must be converted to numeric repr before serialized.
        """
        _lst_payload = list(_nt_payload)

        if (len(_lst_payload) > len(msg._fields)):
            raise ValueError("Number of members in the payload {} must be less or equal to number of \
                             fields in message {}".format(len(_lst_payload),len(msg._fields)))
        
        numPadBytes = 0

        # Message and payload should have the same number of elements
        for i, field in enumerate(msg._fields):
            print(i, _lst_payload[i-numPadBytes])
            if isinstance(field, core.PadByte):
                numPadBytes +=1

            if isinstance(field, core.BitField):
                _val = Payload.evaluate_BitFields(field,_lst_payload[i-numPadBytes])
                _lst_payload[i] = _val

        return Payload.serialize(msg, _lst_payload)


    @staticmethod
    def evaluate_BitFields(bit_field:core.BitField, _nt):
        """
        Evaluate the value stored inside a namedtuple which represents a bit-field.
        :param _nt          a named tuple instance with bitfield seperated by flag values
        param:bit_field     core.BitField instancewhich contains the description of the named tuple
                            (These are members of ***_MSG described in 'sparkfun_predefine' 
                            e.g. CFG_CLS_PRT_UART_MSG._fields[2]) 

        :return:            Sum of values stored by seperated members of the bitfield
        """
        _members = list(_nt)
        _val = 0

        for mem, sf in zip(_members, bit_field._subfields):
            _val +=  mem << sf._start

        return _val
        

class NMEACfg:

    __slots__ = ['name', 'rate']
    
    
    NMEA_MSG_CLS_ID =  {'DTM': [0xF0, 0x0A], 'GBQ': [0xF0, 0x44],
                'GBS': [0xF0, 0x09], 'GGA': [0xF0, 0x00],
                'GLL': [0xF0, 0x01], 'GLQ': [0xF0, 0x43],
                'GNQ': [0xF0, 0x42], 'GNS': [0xF0, 0x0D],
                'GPQ': [0xF0, 0x40], 'GRS': [0xF0, 0x06],
                'GSA': [0xF0, 0x02], 'GST': [0xF0, 0x07],
                'GSV': [0xF0, 0x03], 'RMC': [0xF0, 0x04],
                'THS': [0xF0, 0x0E], 'TXT': [0xF0, 0x41],
                'VLW': [0xF0, 0x0F], 'VTG': [0xF0, 0x05],
                'ZDA': [0xF0, 0x08], 'CONFIG': [0xF1, 0x41],
                'POSITION': [0xF1, 0x41], 'RATE': [0xF1, 0X40],
                'SVSTATUS': [0xF1, 0x03], 'TIME': [0xF1, 0x04] }
                

    @staticmethod
    def enable_msg(dev_gps: UbloxGps, msg: str, rate: int):
        payload = Payload.serialize(sp.CFG_CLS_MSG_MSG, [NMEACfg.NMEA_MSG_CLS_ID[msg][0], 
                                                NMEACfg.NMEA_MSG_CLS_ID[msg][1],
                rate])
        
        dev_gps.send_message(sp.CFG_CLS, CFG_MSGS.get('MSG'), payload)

        parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
        cls_name, msg_name, payload = parse_tool.receive_from(dev_gps.hard_port)
        ack_ = parse_tool.receive_from(dev_gps.hard_port)

        return ack_[1] == 'ACK'
                                                                    
    
    @staticmethod
    def disable_all(dev_gps: UbloxGps):
        for key, val in NMEACfg.NMEA_MSG_CLS_ID.items():
            payload = Payload.serialize(sp.CFG_CLS_MSG_MSG, [val[0], 
                                                val[1], 0])
            dev_gps.send_message(sp.CFG_CLS, CFG_MSGS.get('MSG'), payload)
            
            parse_tool = core.Parser([sp.CFG_CLS, sp.ACK_CLS])
            
            cls_name, msg_name, payload = parse_tool.receive_from(dev_gps.hard_port)
            ack_ = parse_tool.receive_from(dev_gps.hard_port)

        return ack_.name == 'ACK'

