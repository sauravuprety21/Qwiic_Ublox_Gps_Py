"""Predefined message classes"""

from . import core
import struct


__all__ = ['ACK_CLS', 'CFG_CLS', 'ESF_CLS', 'INF_CLS', 'MGA_CLS', 'MON_CLS',
           'NAV_CLS', 'TIM_CLS', 'NMEACfg',]


ACK_CLS_ACK_MSG = core.Message(0x01, 'ACK', [
                            core.Field('clsID', 'U1'),
                            core.Field('msgID', 'U1'),
                    ])
ACK_CLS_NACK_MSG =  core.Message(0x00, 'NAK', [
                            core.Field('clsID', 'U1'),
                            core.Field('msgID', 'U1'),
                    ])

CFG_CLS_MSG_MSG =  core.Message(0x01, 'MSG',[
                        core.Field('msgClass', 'U1'),
                        core.Field('msgID', 'U1'),
                        core.Field('rate', 'U1')
                    ])

CFG_CLS_OTP_MSG = core.Message(0x41, 'OTP', [
                    ])

CFG_CLS_PIO_MSG = core.Message(0x2C, 'PIO', [
                                core.Field('version', 'U1'),
                                core.Field('request', 'U1'),
                                core.RepeatedBlock('RB', [ 
                                    core.Field('requiredPinState', 'U1'),
                                ]) 
                    ])

CFG_CLS_PRT_MSG = core.Message(0x00, 'PRT', [
                        core.Field('portID', 'U1'),
                        core.PadByte(repeat=1),
                        core.BitField('txReady', 'X2', [
                            core.Flag('en', 0, 1),
                            core.Flag('pol', 1, 2),
                            core.Flag('pin', 2, 6),
                            core.Flag('thres', 7, 16),
                        ]),
                        core.BitField('mode', 'X4', [
                            core.Flag('charLen', 6, 8),
                            core.Flag('parity', 9, 12),
                            core.Flag('nStopBits', 12, 13),
                        ]),
                        core.Field('baudRate', 'U4'),
                        core.BitField('inProtoMask', 'X2', [
                            core.Flag('inUbx', 0, 1),
                            core.Flag('inNmea', 1, 2),
                            core.Flag('inRtcm', 2, 3),
                            core.Flag('inRtcm3', 5, 6),
                        ]),
                        core.BitField('outProtoMask', 'X2', [
                            core.Flag('outUbx', 0, 1),
                            core.Flag('outNmea', 1, 2),
                            core.Flag('outRtcm3', 5, 6),
                        ]),
                        core.BitField('flags', 'X2', [
                            core.Flag('extendedTxTimeout', 0, 1),
                        ]),
                        core.PadByte(repeat=2)
                    ])

CFG_CLS_PT2_MSG = core.Message(0x59, 'PT2', [
                        core.Field('version', 'U1'),
                        core.BitField('activate', 'X1', [
                            core.Flag('enable', 0, 1),
                            core.Flag('lnaMode', 6, 8),
                        ]),
                        core.Field('extint', 'U1'),
                        core.Field('reAcqCno', 'U1'),
                        core.Field('refFreq', 'U4'),
                        core.Field('refFreqAcc', 'U4'),
                        core.RepeatedBlock('RB', [ 
                            core.Field('gnssId', 'U1'),
                            core.Field('svId', 'U1'),
                            core.Field('sigId', 'U1'),
                            core.Field('accsId', 'U1'),
                        ]) 
                    ])

CFG_CLS_RST_MSG = core.Message(0x04, 'RST', [
                        core.BitField('navBbrMask', 'X2', [
                            core.Flag('eph', 0, 1),
                            core.Flag('alm', 1, 2),
                            core.Flag('health', 2, 3),
                            core.Flag('klob', 3, 4),
                            core.Flag('pos', 4, 5),
                            core.Flag('clkd', 5, 6),
                            core.Flag('osc', 6, 7),
                            core.Flag('utc', 7, 8),
                            core.Flag('rtc', 8, 9),
                            core.Flag('sfdr', 11, 12),
                            core.Flag('vmon', 12, 13),
                            core.Flag('tct', 13, 14),
                            core.Flag('aop', 15, 16),
                        ]),
                        core.Field('resetMode', 'U1'),
                        core.PadByte(repeat=1),
                    ])

CFG_CLS_SPT_MSG = core.Message(0x64, 'SPT', [
                        core.Field('version', 'U1'),
                        core.PadByte(repeat=1),
                        core.Field('sensorId', 'U2'),
                        core.PadByte(repeat=8),
                    ])

CFG_CLS_USBTEST_MSG = core.Message(0x58, 'USBTEST', [
                        core.Field('version', 'U1'),
                        core.Field('usbPinState', 'U1'),
                    ])

CFG_CLS_VALDEL_MSG = core.Message(0x8c, 'VALDEL', [ # With transaction
                        core.Field('version', 'U1'),
                        core.Field('usbPinState', 'U1'),
                        core.BitField('layers', 'X1', [
                            core.Flag('bbr', 1, 2),
                            core.Flag('flash', 2, 3),
                        ]),
                        core.PadByte(repeat=2),
                        core.RepeatedBlock('RB', [
                            core.Field('keys','U4'),
                        ]),
                    ])

CFG_CLS_VALGET_MSG = core.Message(0x8b, 'VALGET', [ # Get configuration items
                        core.Field('version', 'U1'),
                        core.Field('layer', 'U1'),
                        core.Field('position', 'U2'),
                        core.RepeatedBlock('RB', [
                            core.Field('cfgData','U4'),
                        ]),
                    ])

CFG_CLS_VALSET_MSG = core.Message(0x8a, 'VALSET', [ # With Tranaction 
                        core.Field('version', 'U1'),
                        core.BitField('layers', 'X1', [
                            core.Flag('ram', 0, 1),
                            core.Flag('bbr', 1, 2),
                            core.Flag('flash', 2, 3),
                        ]),
                        core.Field('transaction', 'U1'),
                        core.Field('action', 'U1'),
                        core.PadByte(repeat=1),
                        core.RepeatedBlock('RB', [
                            core.Field('cfgData','U4'),
                        ]),
                    ])

INF_CLS_DEBUG_MSG = core.Message(0x04, 'DEBUG', [
                        core.RepeatedBlock('RB', [
                            core.Field('str','C'),
                        ]),
                    ])

INF_CLS_ERROR_MSG = core.Message(0x00, 'ERROR', [
                        core.RepeatedBlock('RB', [
                            core.Field('str','C'),
                        ]),
                    ])

INF_CLS_NOTICE_MSG = core.Message(0x02, 'NOTICE', [
                        core.RepeatedBlock('RB', [
                            core.Field('str','C'),
                        ]),
                    ])

INF_CLS_TEST_MSG =  core.Message(0x03, 'TEST', [
                        core.RepeatedBlock('RB', [
                            core.Field('str','C'),
                        ]),
                    ])

INF_CLS_WARNING_MSG = core.Message(0x01, 'WARNING', [
                        core.RepeatedBlock('RB', [
                            core.Field('str','C'),
                        ]),
                    ])


ACK_CLS = core.Cls(0x05, 'ACK', 
                   [
                    ACK_CLS_ACK_MSG,
                    ACK_CLS_NACK_MSG
                    ])


CFG_CLS = core.Cls(0x06, 'CFG', 
                    [
                    CFG_CLS_MSG_MSG,
                    CFG_CLS_OTP_MSG,
                    CFG_CLS_PIO_MSG,    
                    CFG_CLS_PRT_MSG,    
                    CFG_CLS_PT2_MSG,
                    CFG_CLS_RST_MSG,    
                    CFG_CLS_SPT_MSG,    
                    CFG_CLS_USBTEST_MSG,    
                    CFG_CLS_VALDEL_MSG,    
                    CFG_CLS_VALGET_MSG,    
                    CFG_CLS_VALSET_MSG,    
])

MGA_CLS_ACK_MSG = core.Message(0x60, 'ACK', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.Field('infoCode', 'U1'),
                    core.Field('msgId', 'U1'),
                    core.Field('msgPayloadStart', 'U1'),
                ])

MGA_CLS_BDS_EPH_MSG = core.Message(0x03, 'BDS_EPH', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.Field('svId', 'U1'),
                    core.PadByte(repeat=1),
                    core.Field('SatH1', 'U1'),
                    core.Field('IODC', 'U1'),
                    core.Field('a2', 'I2'),
                    core.Field('a1', 'I4'),
                    core.Field('a0', 'I4'),
                    core.Field('toc', 'U4'),
                    core.Field('TGD1', 'I2'),
                    core.Field('URAI', 'U1'),
                    core.Field('IODE', 'U1'),
                    core.Field('toe', 'U4'),
                    core.Field('sqrtA', 'U4'),
                    core.Field('e', 'U4'),
                    core.Field('omega', 'I4'),
                    core.Field('Deltan', 'I2'),
                    core.Field('IDOT', 'I2'),
                    core.Field('M0', 'I4'),
                    core.Field('Omega0', 'I4'),
                    core.Field('OmegaDot', 'I4'),
                    core.Field('i0', 'I4'),
                    core.Field('Cuc', 'I4'),
                    core.Field('Cus', 'I4'),
                    core.Field('Crc', 'I4'),
                    core.Field('Crs', 'I4'),
                    core.PadByte(repeat=4),
                ])

MGA_CLS_BDS_ALM_MSG = core.Message(0x03, 'BDS_ALM', [
                            core.Field('type', 'U1'),
                            core.Field('version', 'U1'),
                            core.Field('svId', 'U1'),
                            core.PadByte(repeat=1),
                            core.Field('Wna', 'U1'),
                            core.Field('toa', 'U1'),
                            core.Field('deltaI', 'I2'),
                            core.Field('sqrtA', 'U4'),
                            core.Field('omega', 'I4'),
                            core.Field('M0', 'I4'),
                            core.Field('Omega0', 'I4'),
                            core.Field('OmegaDot', 'I4'),
                            core.Field('a0', 'I4'),
                            core.PadByte(repeat=4),
                ])

MGA_CLS_BDS_HEALTH_MSG = core.Message(0x03, 'BDS_HEALTH', [
                            core.Field('type', 'U1'),
                            core.Field('version', 'U1'),
                            core.PadByte(repeat=2),
                            core.Field('healthCode', 'U2'),
                            core.PadByte(repeat=4),
                        ])

MGA_CLS_BDS_UTC_MSG1 = core.Message(0x03, 'BDS_UTC', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.PadByte(repeat=2),
                    core.Field('a0UTC', 'I4'),
                    core.Field('a1UTC', 'I4'),
                    core.Field('dtLS', 'I1'),
                    core.PadByte(repeat=1),
                    core.Field('wnRec', 'U1'),
                    core.Field('wnLSF', 'U1'),
                    core.Field('dN', 'U1'),
                    core.Field('dtLSF', 'U1'),
                    core.PadByte(repeat=2),
                ])
MGA_CLS_BDS_UTC_MSG2 = core.Message(0x03, 'BDS_UTC', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.PadByte(repeat=2),
                    core.Field('alpha0', 'I1'),
                    core.Field('alpha1', 'I1'),
                    core.Field('alpha2', 'I1'),
                    core.Field('alpha3', 'I1'),
                    core.Field('beta0', 'I1'),
                    core.Field('beta1', 'I1'),
                    core.Field('beta2', 'I1'),
                    core.Field('beta3', 'I1'),
                    core.PadByte(repeat=4),
                ])

MGA_CLS_DBD_POLL_MSG = core.Message(0x80, 'DBD_POLL', [
                ])

MGA_CLS_DBD_IO_MSG = core.Message(0x80, 'DBD_IO', [
                    core.PadByte(repeat=12),
                    core.RepeatedBlock('RB', [
                        core.Field('data','U1'),
                    ]),
                ])

MGA_CLS_GAL_EPH_MSG = core.Message(0x02, 'GAL_EPH', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.Field('svId', 'U1'),
                    core.PadByte(repeat=1),
                    core.Field('iodNav', 'U2'),
                    core.Field('deltaN', 'I2'),
                    core.Field('m0', 'I4'),
                    core.Field('e', 'U4'),
                    core.Field('sqrtA', 'U4'),
                    core.Field('omega0', 'I4'),
                    core.Field('i0', 'I4'),
                    core.Field('omega', 'I4'),
                    core.Field('omegaDot', 'I4'),
                    core.Field('iDot', 'I2'),
                    core.Field('cuc', 'I2'),
                    core.Field('cus', 'I2'),
                    core.Field('crc', 'I2'),
                    core.Field('crs', 'I2'),
                    core.Field('cis', 'I2'),
                    core.Field('toe', 'U2'),
                    core.Field('af0', 'I4'),
                    core.Field('af1', 'I4'),
                    core.Field('af2', 'I1'),
                    core.Field('sisaIndexE1E5b', 'U1'),
                    core.Field('toc', 'U2'),
                    core.Field('bgdE1E5b', 'I2'),
                    core.PadByte(repeat=2),
                    core.Field('healthE1B', 'U1'),
                    core.Field('dataValidityE5b', 'U1'),
                    core.PadByte(repeat=4),
                ])

MGA_CLS_GAL_ALM_MSG = core.Message(0x02, 'GAL_ALM', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.Field('svId', 'U1'),
                    core.PadByte(repeat=1),
                    core.Field('ioda', 'U1'),
                    core.Field('almWnA', 'U1'),
                    core.Field('toa', 'U2'),
                    core.Field('deltaSqrtA', 'I2'),
                    core.Field('e', 'U2'),
                    core.Field('deltaI', 'I2'),
                    core.Field('omega0', 'I2'),
                    core.Field('omegaDot', 'I2'),
                    core.Field('omega', 'I2'),
                    core.Field('m0', 'I2'),
                    core.Field('af0', 'I2'),
                    core.Field('healthE1B', 'U1'),
                    core.Field('healthE5b', 'U1'),
                    core.PadByte(repeat=4),
                ])

MGA_CLS_GAL_TIMEOFFSET_MSG = core.Message(0x02, 'GAL_TIMEOFFSET', [
                                core.Field('type', 'U1'),
                                core.Field('version', 'U1'),
                                core.PadByte(repeat=2),
                                core.Field('a0G', 'I2'),
                                core.Field('a1G', 'I2'),
                                core.Field('t0G', 'U1'),
                                core.Field('wn0G', 'U1'),
                                core.PadByte(repeat=2),
                        ])

MGA_CLS_GAL_UTC_MSG = core.Message(0x02, 'GAL_UTC', [
                    core.Field('type', 'U1'),
                    core.Field('version', 'U1'),
                    core.PadByte(repeat=2),
                    core.Field('a0', 'I4'),
                    core.Field('a1', 'I4'),
                    core.Field('dtLS', 'I1'),
                    core.Field('tot', 'U1'),
                    core.Field('wnt', 'U1'),
                    core.Field('wnLSF', 'U1'),
                    core.Field('dN', 'U1'),
                    core.Field('dTLSF', 'I1'),
                    core.PadByte(repeat=2),
                ])

MON_CLS_COMMS_MSG = core.Message(0x36, 'COMMS', [
                    core.Field('version', 'U1'),
                    core.Field('nPorts', 'U1'),
                    core.BitField('txErrors', 'X1', [
                        core.Flag('mem', 0, 1),
                        core.Flag('alloc', 1, 2),
                    ]),
                    core.PadByte(repeat=0),
                    core.Field('protIds', 'U1'),
                    core.RepeatedBlock('RB', [
                        core.Field('portId','U2'),
                        core.Field('txPending','U2'),
                        core.Field('txBytes','U4'),
                        core.Field('txUsage','U1'),
                        core.Field('txPeakUsage','U1'),
                        core.Field('rxPending','U2'),
                        core.Field('rxBytes','U4'),
                        core.Field('rxUsage','U1'),
                        core.Field('rxPeakUsage','U1'),
                        core.Field('overrunErrs','U2'),
                        core.Field('msgs','U2'),
                        core.PadByte(repeat=0),
                        core.Field('skipped', 'U4'),
                    ]),
                ])

MON_CLS_GNSS_MSG = core.Message(0x28, 'GNSS', [
                        core.Field('version', 'U1'),
                        core.BitField('supported', 'X1', [
                            core.Flag('GPSSup', 0, 1),
                            core.Flag('GlonassSup', 1, 2),
                            core.Flag('BeidouSup', 2, 3),
                            core.Flag('GalileoSup', 3, 4),
                        ]),
                        core.BitField('defaultGnss', 'X1', [
                            core.Flag('GPSDef', 0, 1),
                            core.Flag('GlonassDef', 1, 2),
                            core.Flag('BeidouDef', 2, 3),
                            core.Flag('GalileoDef', 3, 4),
                        ]),
                        core.BitField('enabled', 'X1', [
                            core.Flag('GPSEna', 0, 1),
                            core.Flag('GlonasEna', 1, 2),
                            core.Flag('BeidouEna', 2, 3),
                            core.Flag('GalileoEna', 3, 4),
                        ]),
                        core.Field('simultaneous', 'U1'),
                        core.PadByte(repeat=2),
                ])

MON_CLS_HW3_MSG = core.Message(0x37, 'HW3', [ #HW and HW2 not implemented
                    core.Field('version', 'U1'),
                    core.Field('nPins', 'U1'),
                    core.BitField('flags', 'X1', [
                        core.Flag('rtcCalib', 0, 1),
                        core.Flag('safeBoot', 1, 2),
                        core.Flag('xtalAbsent', 2, 3),
                    ]),
                    core.Field('hwVersion', 'C'),
                    core.PadByte(repeat=0),
                    core.RepeatedBlock('RB', [
                        core.Field('pinId', 'U2'),
                        core.BitField('pinMask', 'X2', [
                            core.Flag('periphPIO', 0, 1),
                            core.Flag('pinBank', 1, 4),
                            core.Flag('direction', 4, 5),
                            core.Flag('value', 5, 6),
                            core.Flag('vpManager', 6, 7),
                            core.Flag('pioIrq', 7, 8),
                            core.Flag('pioPullHigh', 8, 9),
                            core.Flag('pioPullLow', 9, 10),
                        ]),
                        core.Field('VP', 'U1'),
                        core.PadByte(repeat=0),
                    ]),
                ])
MON_CLS_PATCH_MSG = core.Message(0x27, 'PATCH', [ 
                    core.Field('version', 'U2'),
                    core.Field('nEntries', 'U2'),
                    core.RepeatedBlock('RB', [
                        core.BitField('patchInfo', 'X4', [
                            core.Flag('activated', 0, 1),
                            core.Flag('location', 1, 3),
                        ]),
                        core.Field('comparatorNumber', 'U4'),
                        core.Field('patchAddress', 'U4'),
                        core.Field('patchData', 'U4'),
                    ]),
                ])

MON_CLS_PIO_MSG = core.Message(0x24, 'PIO', [ 
                    core.Field('version', 'U1'),
                    core.Field('responseType', 'U1'),
                    core.RepeatedBlock('RB', [
                        core.Field('pinState', 'U1'),
                    ]),
                ])
MON_CLS_PT2_MSG =  core.Message(0x2b, 'PT2', [ 
                    core.Field('version', 'U1'),
                    core.Field('testmode', 'U1'),
                    core.Field('numRfChn', 'U1'),
                    core.Field('numSvSigDesc', 'U1'),
                    core.Field('testRunTime', 'U4'),
                    core.Field('clkDriftAid', 'I4'),
                    core.Field('clkDriftTrk', 'I4'),
                    core.Field('rtcFreq', 'U4'),
                    core.Field('postStatus', 'U4'),
                    core.RepeatedBlock('RB', [
                        core.Field('rfPga', 'U1'),
                        core.PadByte(repeat=27), #? 
                    ]),
                    #core.RepeatedBlock('RB2', [
                    #    core.Field('gnssId', 'U1'),
                    #    core.Field('svId', 'U1'),
                    #    core.Field('sigId', 'U1'),
                    #    core.Field('accsId', 'U1'),
                    #    core.Field('cnoMin', 'U2'),
                    #    core.Field('cnoMax', 'U2'),
                    #    core.PadByte(repeat=14), #?
                    #    core.Field('carrPhDevMax', 'U1'),
                    #    core.BitField('signalInfo', 'X1', [
                    #        core.Flag('ifChnValid', 0, 1),
                    #        core.Flag('ifChnId', 1, 3),
                    #    ]),
                    #    core.Field('codeLockSuccess', 'U1'),
                    #    core.Field('phaseLockSuccess', 'U1'),
                    #    core.Field('minCodeLockTime', 'U2'),
                    #    core.Field('maxCodeLockTime', 'U2'),
                    #    core.Field('minPhaseLockTime', 'U2'),
                    #    core.Field('maxPhaseLockTime', 'U2'),
                    #    core.PadByte(repeat=2),
                    #]),
                ])
MON_CLS_RF_MSG = core.Message(0x38, 'RF', [ 
                    core.Field('version', 'U1'),
                    core.Field('nBlocks', 'U1'),
                    core.PadByte(repeat=1),
                    core.RepeatedBlock('RB', [
                        core.Field('blockId', 'U1'),
                        core.BitField('flags', 'X1', [
                            core.Flag('jammingState', 0, 2),
                        ]),
                        core.Field('antStatus', 'U1'),
                        core.Field('antPower', 'U1'),
                        core.Field('postStatus', 'U4'),
                        core.PadByte(repeat=3),
                        core.Field('noisePerMS', 'U2'),
                        core.Field('agcCnt', 'U2'),
                        core.Field('jamInd', 'U1'),
                        core.Field('ofsI', 'I1'),
                        core.Field('magI', 'U1'),
                        core.Field('ofsQ', 'I1'),
                        core.Field('magQ', 'U1'),
                        core.PadByte(repeat=2),
                    ]),
                ])

MON_CLS_RXR_MSG = core.Message(0x21, 'RXR', [ 
                    core.BitField('flags', 'X1', [
                        core.Flag('awake', 0, 1),
                    ]),
                ])

MON_CLS_SPT_MSG = core.Message(0x2f, 'SPT', [ 
                    core.Field('version', 'U1'),
                    core.Field('numSensor', 'U1'),
                    core.Field('numRes', 'U1'),
                    core.PadByte(repeat=1),
                    core.RepeatedBlock('RB', [
                        core.Field('sensorId', 'U1'),
                        core.BitField('drvVer', 'X1', [
                            core.Flag('drvVerMaj', 0, 4),
                            core.Flag('drvVerMin', 4, 7),
                        ]),
                        core.Field('testState', 'U1'),
                        core.Field('drvFileName', 'U1'),
                    ]),
                    #core.RepeatedBlock('RB2', [
                    #    core.Field('sensorIdRes', 'U2'),
                    #    core.Field('sensorType', 'U2'),
                    #    core.Field('resType', 'U2'),
                    #    core.PadByte(repeat=2),
                    #    core.Field('value', 'I4'),
                    #]),
                ]) 

NAV_CLS_ATT_MSG = core.Message(0x05, 'ATT', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.PadByte(repeat=1),
                        core.Field('roll', 'I4'),
                        core.Field('pitch', 'I4'),
                        core.Field('heading', 'I4'),
                        core.Field('accRoll', 'U4'),
                        core.Field('accPitch', 'U4'),
                        core.Field('accHeading', 'U4'),
                    ])

NAV_CLS_CLOCK_MSG = core.Message(0x22, 'CLOCK', [
                        core.Field('iTOW', 'U4'),
                        core.Field('clkB', 'I4'),
                        core.Field('clkD', 'I4'),
                        core.Field('tAcc', 'U4'),
                        core.Field('fAcc', 'U4'),
                    ])

NAV_CLS_COV_MSG = core.Message(0x36, 'COV', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.Field('posCovValid', 'U1'),
                        core.Field('velCovValid', 'U1'),
                        core.PadByte(repeat=6),
                        core.Field('posCovNN', 'R4'),
                        core.Field('posCovNE', 'R4'),
                        core.Field('posCovND', 'R4'),
                        core.Field('posCovEE', 'R4'),
                        core.Field('posCovED', 'R4'),
                        core.Field('posCovDD', 'R4'),
                        core.Field('velCovNN', 'R4'),
                        core.Field('velCovNE', 'R4'),
                        core.Field('velCovND', 'R4'),
                        core.Field('velCovEE', 'R4'),
                        core.Field('velCovED', 'R4'),
                        core.Field('velCovDD', 'R4'),
                    ])

NAV_CLS_DOP_MSG = core.Message(0x04, 'DOP', [
                        core.Field('iTOW', 'U4'),
                        core.Field('gDOP', 'U2'),
                        core.Field('pDOP', 'U2'),
                        core.Field('tDOP', 'U2'),
                        core.Field('vDOP', 'U2'),
                        core.Field('hDOP', 'U2'),
                        core.Field('nDOP', 'U2'),
                        core.Field('eDOP', 'U2'),
                    ])

NAV_CLS_EELL_MSG = core.Message(0x3d, 'EELL', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.Field('reserved', 'U1'),
                        core.Field('errEllipseOrient', 'U2'),
                        core.Field('errEllipseMajor', 'U4'),
                        core.Field('errEllipseMinor', 'U4'),
                    ])

NAV_CLS_EOE_MSG = core.Message(0x61, 'EOE', [
                        core.Field('iTOW', 'U4'),
                    ])

NAV_CLS_GEOFENCE_MSG = core.Message(0x39, 'GEOFENCE', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.Field('status', 'U1'),
                        core.Field('numFences', 'U1'),
                        core.Field('combState', 'U1'),
                        core.RepeatedBlock('RB', [
                            core.Field('state', 'U1'),
                            core.Field('id', 'U1'),
                        ]),
                    ])

NAV_CLS_HPPOSECEF_MSG = core.Message(0x13, 'HPPOSECEF', [
                        core.Field('version', 'U1'),
                        core.PadByte(repeat=1),
                        core.Field('iTOW', 'U4'),
                        core.Field('ecefX', 'I4'),
                        core.Field('ecefY', 'I4'),
                        core.Field('ecefZ', 'I4'),
                        core.Field('ecefXHp', 'I1'),
                        core.Field('ecefYHp', 'I1'),
                        core.Field('ecefZHp', 'I1'),
                        core.BitField('flags', 'X1', [
                            core.Flag('invalidEcef', 0 ,1),
                        ]),
                        core.Field('pAcc', 'U4'),
                    ])

NAV_CLS_HPPOSLLH_MSG = core.Message(0x14, 'HPPOSLLH', [
                        core.Field('version', 'U1'),
                        core.PadByte(repeat=1),
                        core.BitField('flags', 'X1', [
                            core.Flag('invalidLh', 0 ,1),
                        ]),
                        core.Field('iTOW', 'U4'),
                        core.Field('lon', 'I4'),
                        core.Field('lat', 'I4'),
                        core.Field('height', 'I4'),
                        core.Field('hMSL', 'I4'),
                        core.Field('lonHp', 'I1'),
                        core.Field('latHp', 'I1'),
                        core.Field('heightHp', 'I1'),
                        core.Field('hMSLHp', 'I1'),
                        core.Field('hAcc', 'U4'),
                        core.Field('vAcc', 'U4'),
                    ])

NAV_CLS_ORB_MSG = core.Message(0x34, 'ORB', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.Field('numSv', 'U1'),
                        core.PadByte(repeat=1),
                        core.RepeatedBlock('RB', [
                            core.Field('gnssId', 'U1'),
                            core.Field('svId', 'U1'),
                            core.BitField('svFlag', 'X1', [
                                core.Flag('health', 0, 2),
                                core.Flag('visibility', 2, 4),
                            ]),
                            core.BitField('eph', 'X1', [
                                core.Flag('ephUsability', 0, 5),
                                core.Flag('ephSource', 5, 8),
                            ]),
                            core.BitField('alm', 'X1', [
                                core.Flag('almUsability', 0, 5),
                                core.Flag('almSource', 5, 8),
                            ]),
                            core.BitField('otherOrb', 'X1', [
                                core.Flag('anoAopUsability', 0, 5),
                                core.Flag('type', 5, 8),
                            ]),
                        ]),
                    ])

NAV_CLS_POSECEF_MSG = core.Message(0x01, 'POSECEF', [
                        core.Field('iTOW', 'U4'),
                        core.Field('ecefX', 'I4'),
                        core.Field('ecefY', 'I4'),
                        core.Field('ecefZ', 'I4'),
                        core.Field('pAcc', 'U4'),
                    ])

NAV_CLS_POSLLH_MSG = core.Message(0x02, 'POSLLH', [
                        core.Field('iTOW', 'U4'),
                        core.Field('lon', 'I4'),
                        core.Field('lat', 'I4'),
                        core.Field('height', 'I4'),
                        core.Field('hMSL', 'I4'),
                        core.Field('hAcc', 'U4'),
                        core.Field('vAcc', 'U4'),
                    ])

NAV_CLS_PVT_MSG = core.Message(0x07, 'PVT', [
                        core.Field('iTOW', 'U4'),
                        core.Field('year', 'U2'),
                        core.Field('month', 'U1'),
                        core.Field('day', 'U1'),
                        core.Field('hour', 'U1'),
                        core.Field('min', 'U1'),
                        core.Field('sec', 'U1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('validDate', 0, 1),
                            core.Flag('validTime', 1, 2),
                            core.Flag('fullyResolved', 2, 3),
                            core.Flag('validMag', 3, 4),
                        ]),
                        core.Field('tAcc', 'U4'),
                        core.Field('nano', 'I4'),
                        core.Field('fixType', 'U1'),
                        core.BitField('flags', 'X1', [
                            core.Flag('gnssFixOK', 0, 1),
                            core.Flag('diffSoln', 1, 2),
                            core.Flag('psmState', 2, 5),
                            core.Flag('headVehValid', 5, 6),
                            core.Flag('carrSoln', 6, 8),
                        ]),
                        core.BitField('flags2', 'X1', [
                            core.Flag('confirmedAvai', 5, 6),
                            core.Flag('confirmedDate', 6, 7),
                            core.Flag('confirmedTime', 7, 8),
                        ]),
                        core.Field('numSV', 'U1'),
                        core.Field('lon', 'I4'),
                        core.Field('lat', 'I4'),
                        core.Field('height', 'I4'),
                        core.Field('hMSL', 'I4'),
                        core.Field('hAcc', 'U4'),
                        core.Field('vAcc', 'U4'),
                        core.Field('velN', 'I4'),
                        core.Field('velE', 'I4'),
                        core.Field('velD', 'I4'),
                        core.Field('gSpeed', 'I4'),
                        core.Field('headMot', 'I4'),
                        core.Field('sAcc', 'U4'),
                        core.Field('headAcc', 'U4'),
                        core.Field('pDOP', 'U2'),
                        core.BitField('flags3', 'X1', [
                            core.Flag('invalidL1h', 0, 1),
                        ]),
                        core.PadByte(repeat=4),
                        core.Field('headVeh', 'I4'),
                        core.Field('magDec', 'I2'),
                        core.Field('magAcc', 'U2'),
                    ])

NAV_CLS_RELPOSNED_MSG = core.Message(0x3C, 'RELPOSNED', [
                        core.Field('version', 'U1'),
                        core.PadByte(repeat=0),
                        core.Field('refStationId', 'U2'),
                        core.Field('iTOW', 'U4'),
                        core.Field('relPosN', 'I4'),
                        core.Field('relPosE', 'I4'),
                        core.Field('relPosD', 'I4'),
                        core.Field('relPosLength', 'I4'),
                        core.Field('relPosHeading', 'I4'),
                        core.PadByte(repeat=1),
                        core.Field('relPosHPN', 'I1'),
                        core.Field('relPosHPE', 'I1'),
                        core.Field('relPosHPD', 'I1'),
                        core.Field('relPosHPLength', 'I1'),
                        core.Field('accN', 'U4'),
                        core.Field('accE', 'U4'),
                        core.Field('accD', 'U4'),
                        core.Field('accLength', 'U4'),
                        core.Field('accHeading', 'U4'),
                        core.PadByte(repeat=1),
                        core.BitField('flags', 'X4', [
                            core.Flag('gnssFixOK', 0, 1),
                            core.Flag('diffSoln', 1, 2),
                            core.Flag('relPosValid', 2, 3),
                            core.Flag('carrSoln', 3, 5),
                            core.Flag('isMoving', 5, 6),
                            core.Flag('refPosMiss', 6, 7),
                            core.Flag('refObsMiss', 7, 8),
                            core.Flag('relPosHeadingValid', 8, 9),
                            core.Flag('relPosNormalized', 9, 10),
                        ]),
                    ])

NAV_CLS_SAT_MSG = core.Message(0x35, 'SAT', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.Field('numSvs', 'U1'),
                        core.PadByte(repeat=1),
                        core.RepeatedBlock('RB', [
                            core.Field('gnssId', 'U1'),
                            core.Field('svId', 'U1'),
                            core.Field('cno', 'U1'),
                            core.Field('elev', 'I1'),
                            core.Field('azim', 'I2'),
                            core.Field('prRes', 'I2'),
                            core.BitField('flags', 'X4', [
                                core.Flag('qualityInd', 0, 3),
                                core.Flag('svUsed', 3, 4),
                                core.Flag('health', 4, 6),
                                core.Flag('diffCorr', 6, 7),
                                core.Flag('smoothed', 7, 8),
                                core.Flag('orbitSource', 8, 11),
                                core.Flag('ephAvail', 11, 12),
                                core.Flag('almAvail', 12, 13),
                                core.Flag('anoAvail', 13, 14),
                                core.Flag('aopAvail', 14, 15),
                                core.Flag('sbasCorrUsed', 16, 17),
                                core.Flag('rtcmCorrUsed', 17, 18),
                                core.Flag('slasCorrUsed', 18, 19),
                                core.Flag('prCorrUsed', 20, 21),
                                core.Flag('crCorrUsed', 21, 22),
                                core.Flag('doCorrUsed', 22, 23),
                            ]),
                        ]),
                    ])

NAV_CLS_SBAS_MSG = core.Message(0x32, 'SBAS', [
                        core.Field('iTOW', 'U4'),
                        core.Field('geo', 'U1'),
                        core.Field('mode', 'U1'),
                        core.Field('sys', 'I1'),
                        core.BitField('service', 'X1', [
                            core.Flag('Ranging', 0, 1),
                            core.Flag('Corrections', 1, 2),
                            core.Flag('Integrity', 2, 3),
                            core.Flag('TestMode', 3, 4),
                            core.Flag('Bad', 4, 5),
                        ]),
                        core.Field('cnt', 'U1'),
                        core.PadByte(repeat=6),
                        core.RepeatedBlock('RB', [
                            core.Field('svid', 'U1'),
                            core.Field('flags', 'U1'),
                            core.Field('udre', 'U1'),
                            core.Field('svSys', 'U1'),
                            core.Field('svService', 'U1'),
                            core.PadByte(repeat=1),
                            core.Field('prc', 'I2'),
                            core.PadByte(repeat=2),
                            core.Field('ic', 'I2'),
                        ]),
                    ])

NAV_CLS_SIG_MSG = core.Message(0x43, 'SIG', [ #here
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.Field('numSigs', 'U1'),
                        core.PadByte(repeat=2),
                        core.RepeatedBlock('RB', [
                            core.Field('gnssId', 'U1'),
                            core.Field('svId', 'U1'),
                            core.Field('sigId', 'U1'),
                            core.Field('freqId', 'U1'),
                            core.Field('prRes', 'I2'),
                            core.Field('cno', 'U1'),
                            core.Field('qualityInd', 'U1'),
                            core.Field('corrSource', 'U1'),
                            core.Field('ionoModel', 'U1'),
                            core.BitField('sigFlags', 'X2', [
                                core.Flag('health', 0, 2),
                                core.Flag('prSmoothed', 2, 3),
                                core.Flag('prUsed', 3, 4),
                                core.Flag('crUsed', 4, 5),
                                core.Flag('doUsed', 5, 6),
                                core.Flag('prCorrUsed', 6, 7),
                                core.Flag('crCorrUsed', 7, 8),
                                core.Flag('doCorrUsed', 8, 9),
                            ]),
                        ]),
                            core.PadByte(repeat=4)
                    ])

NAV_CLS_STATUS_MSG = core.Message(0x03, 'STATUS', [
                        core.Field('iTOW', 'U4'),
                        core.Field('gpsFix', 'U1'),
                        core.BitField('flags', 'X1', [
                            core.Flag('gpsFixOK', 0, 1),
                            core.Flag('diffSoln', 1, 2),
                            core.Flag('wknSet', 2, 3),
                            core.Flag('towSet', 3, 4),
                        ]),
                        core.BitField('fixStat', 'X1', [
                            core.Flag('diffCorr', 0, 1),
                            core.Flag('caarSolnValid', 1, 2),
                            core.Flag('mapMatching', 6, 8),
                        ]),
                        core.BitField('flags2', 'X1', [
                            core.Flag('psmState', 0, 2),
                            core.Flag('spoofDetState', 3, 5),
                            core.Flag('carSoln', 6, 8),
                        ]),
                        core.Field('ttff', 'U4'),
                        core.Field('msss', 'U4'),
                    ])

NAV_CLS_TIMEBDS_MSG = core.Message(0x24, 'TIMEBDS', [
                        core.Field('iTOW', 'U4'),
                        core.Field('SOW', 'U4'),
                        core.Field('fSOW', 'I4'),
                        core.Field('week', 'I2'),
                        core.Field('leapS', 'I1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('sowValid', 0, 1),
                            core.Flag('weekValid', 1, 2),
                            core.Flag('leapSValid', 2, 3),
                        ]),
                        core.Field('tAcc','U4'), 
                    ])

NAV_CLS_TIMEGAL_MSG = core.Message(0x25, 'TIMEGAL', [
                        core.Field('iTOW', 'U4'),
                        core.Field('galTow', 'U4'),
                        core.Field('fGalTow', 'I4'),
                        core.Field('galWno', 'I2'),
                        core.Field('leapS', 'I1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('galValid', 0, 1),
                            core.Flag('galWnoValid', 1, 2),
                            core.Flag('leapSValid', 2, 3),
                        ]),
                        core.Field('tAcc','U4'), 
                    ])

NAV_CLS_TIMEGLO_MSG = core.Message(0x23, 'TIMEGLO', [
                        core.Field('iTOW', 'U4'),
                        core.Field('TOD', 'U4'),
                        core.Field('fTOD', 'I4'),
                        core.Field('Nt', 'U2'),
                        core.Field('N4', 'U1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('todValid', 0, 1),
                            core.Flag('dateValid', 1, 2),
                        ]),
                        core.Field('tAcc','U4'), 
                    ])

NAV_CLS_TIMEGPS_MSG = core.Message(0x20, 'TIMEGPS', [
                        core.Field('iTOW', 'U4'),
                        core.Field('fTOW', 'I4'),
                        core.Field('week', 'I2'),
                        core.Field('leapS', 'I1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('towValid', 0, 1),
                            core.Flag('weekValid', 1, 2),
                            core.Flag('leapSValid', 2, 3),
                        ]),
                        core.Field('tAcc','U4'), 
                    ])

NAV_CLS_TIMELS_MSG = core.Message(0x25, 'TIMELS', [
                        core.Field('iTOW', 'U4'),
                        core.Field('version', 'U1'),
                        core.PadByte(repeat=3),
                        core.Field('srcOfCurrLs', 'U1'),
                        core.Field('currLs', 'I1'),
                        core.Field('srcOfLsChange', 'U1'),
                        core.Field('lsChange', 'I1'),
                        core.Field('timeToLsEvent', 'I4'),
                        core.Field('dateOfLsGpsWn', 'U2'),
                        core.PadByte(repeat=3),
                        core.BitField('valid', 'X1', [
                            core.Flag('validCurrLs', 0, 1),
                            core.Flag('validTimeToLsEvent', 1, 2),
                        ]),
                    ])

NAV_CLS_TIMEQZSS_MSG = core.Message(0x27, 'TIMEQZSS', [
                        core.Field('iTOW', 'U4'),
                        core.Field('qzssTow', 'U4'),
                        core.Field('fQzssTow', 'I4'),
                        core.Field('qzssWno', 'I2'),
                        core.Field('leapS', 'I1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('qzssTowValid', 0, 1),
                            core.Flag('qzssWnoValid', 1, 2),
                            core.Flag('leapSValid', 2, 3),
                        ]),
                        core.Field('tAcc','U4'), 
                    ])

NAV_CLS_TIMEUTC_MSG = core.Message(0x21, 'TIMEUTC', [
                        core.Field('iTOW', 'U4'),
                        core.Field('tAcc', 'U4'),
                        core.Field('nano', 'I4'),
                        core.Field('year', 'U2'),
                        core.Field('month', 'U1'),
                        core.Field('day', 'U1'),
                        core.Field('hour', 'U1'),
                        core.Field('min', 'U1'),
                        core.Field('sec', 'U1'),
                        core.BitField('valid', 'X1', [
                            core.Flag('validTOW', 0, 1),
                            core.Flag('validWKN', 1, 2),
                            core.Flag('validUTC', 2, 3),
                            core.Flag('utcStandard', 4, 8),
                        ]),
                    ])

NAV_CLS_VELECEF_MSG = core.Message(0x11, 'VELECEF', [
                        core.Field('iTOW', 'U4'),
                        core.Field('ecefVX', 'I4'),
                        core.Field('ecefVY', 'I4'),
                        core.Field('ecefVZ', 'I4'),
                        core.Field('sAcc', 'U4'),
                    ])

NAV_CLS_VELNED_MSG = core.Message(0x12, 'VELNED', [
                        core.Field('iTOW', 'U4'),
                        core.Field('velN', 'I4'),
                        core.Field('velE', 'I4'),
                        core.Field('velD', 'I4'),
                        core.Field('speed', 'U4'),
                        core.Field('gSpeed', 'U4'),
                        core.Field('heading', 'I4'),
                        core.Field('sAcc', 'U4'),
                        core.Field('cAcc', 'U4'),
                    ])


TIM_CLS_TM2_MSG = core.Message(0x03, 'TM2', [
                        core.Field('ch', 'U1'),
                        core.BitField('flags', 'X1', [
                            core.Flag('mode', 0, 1),
                            core.Flag('run', 1, 2),
                            core.Flag('newFallingEdge', 2, 3),
                            core.Flag('timeBase', 3, 5),
                            core.Flag('utc', 5, 6),
                            core.Flag('time', 6, 7),
                            core.Flag('newRisingEdge', 7, 8),
                        ]),
                        core.Field('count', 'U2'),
                        core.Field('wnR', 'U2'),
                        core.Field('wnF', 'U2'),
                        core.Field('towMsR', 'U4'),
                        core.Field('towSubMsR', 'U4'),
                        core.Field('towMsF', 'U4'),
                        core.Field('towSubMsF', 'U4'),
                        core.Field('accEst', 'U4'),
                    ])

TIM_CLS_TP_MSG = core.Message(0x01, 'TP', [
                        core.Field('towMS', 'U4'),
                        core.Field('towSubMS', 'U4'),
                        core.Field('qErr', 'I4'),
                        core.Field('week', 'I4'),
                        core.BitField('flags', 'X1', [
                            core.Flag('timeBase', 0, 1),
                            core.Flag('utc', 1, 2),
                            core.Flag('raim', 2, 4),
                            core.Flag('qErrInvalid', 4, 5),
                        ]),
                        core.BitField('refInfo', 'X1', [
                            core.Flag('timeRefGnss', 0, 4),
                            core.Flag('utcStandard', 4, 8),
                        ]),
                    ])

TIM_CLS_VRFY_MSG = core.Message(0x06, 'VRFY', [
                        core.Field('itow', 'I4'),
                        core.Field('frac', 'I4'),
                        core.Field('deltaMs', 'I4'),
                        core.Field('deltaNs', 'I4'),
                        core.Field('wno', 'U2'),
                        core.BitField('flags', 'X1', [
                            core.Flag('src', 0, 3)
                        ]),
                    ])


INF_CLS = core.Cls(0x04, 'INF', 
                [
                INF_CLS_DEBUG_MSG,
                INF_CLS_ERROR_MSG,
                INF_CLS_NOTICE_MSG,
                INF_CLS_TEST_MSG,
                INF_CLS_WARNING_MSG,
])


MGA_CLS = core.Cls(0x13, 'MGA', 
            [
                MGA_CLS_ACK_MSG,
                MGA_CLS_BDS_EPH_MSG,
                MGA_CLS_BDS_ALM_MSG, 
                MGA_CLS_BDS_HEALTH_MSG, 
                MGA_CLS_BDS_UTC_MSG1,
                MGA_CLS_BDS_UTC_MSG2, 
                MGA_CLS_DBD_POLL_MSG, 
                MGA_CLS_DBD_IO_MSG,
                MGA_CLS_GAL_EPH_MSG,
                MGA_CLS_GAL_ALM_MSG, 
                MGA_CLS_GAL_TIMEOFFSET_MSG,
                MGA_CLS_GAL_UTC_MSG,
])


MON_CLS = core.Cls(0x0a, 'MON', 
            [
                MON_CLS_COMMS_MSG, 
                MON_CLS_GNSS_MSG,
                MON_CLS_HW3_MSG,
                MON_CLS_PATCH_MSG,
                MON_CLS_PIO_MSG,
                MON_CLS_PT2_MSG,
                MON_CLS_RF_MSG,
                MON_CLS_RXR_MSG,
                MON_CLS_SPT_MSG,
])


NAV_CLS = core.Cls(0x01, 'NAV', 
                [
                    NAV_CLS_ATT_MSG,
                    NAV_CLS_CLOCK_MSG,
                    NAV_CLS_COV_MSG,
                    NAV_CLS_DOP_MSG,
                    NAV_CLS_EELL_MSG,
                    NAV_CLS_EOE_MSG,
                    NAV_CLS_GEOFENCE_MSG,
                    NAV_CLS_HPPOSECEF_MSG,
                    NAV_CLS_HPPOSLLH_MSG,
                    NAV_CLS_ORB_MSG,
                    NAV_CLS_POSECEF_MSG,
                    NAV_CLS_POSLLH_MSG,
                    NAV_CLS_PVT_MSG,
                    NAV_CLS_RELPOSNED_MSG,
                    NAV_CLS_SAT_MSG,
                    NAV_CLS_SBAS_MSG,
                    NAV_CLS_SIG_MSG,
                    NAV_CLS_STATUS_MSG,
                    NAV_CLS_TIMEBDS_MSG,
                    NAV_CLS_TIMEGAL_MSG,
                    NAV_CLS_TIMEGLO_MSG,
                    NAV_CLS_TIMEGPS_MSG,
                    NAV_CLS_TIMELS_MSG,
                    NAV_CLS_TIMEQZSS_MSG,
                    NAV_CLS_TIMEUTC_MSG,
                    NAV_CLS_VELECEF_MSG,
                    NAV_CLS_VELNED_MSG,
])



TIM_CLS = core.Cls(0x0D, 'TIM', 
            [
                    TIM_CLS_TM2_MSG,
                    TIM_CLS_TP_MSG,
                    TIM_CLS_VRFY_MSG,
])
        
