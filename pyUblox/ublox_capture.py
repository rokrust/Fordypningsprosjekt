#!/usr/bin/env python

import ublox, sys, ephemeris


from optparse import OptionParser

parser = OptionParser("ublox_capture.py [options]")
parser.add_option("--port", help="serial port", default='/dev/ttyACM0')
parser.add_option("--baudrate", type='int',
                  help="serial baud rate", default=115200)
parser.add_option("--log", help="log file", default=None)
parser.add_option("--append", action='store_true', default=False, help='append to log file')
parser.add_option("--reopen", action='store_true', default=False, help='re-open on failure')
parser.add_option("--show", action='store_true', default=False, help='show messages while capturing')
parser.add_option("--dynModel", type='int', default=None, help='set dynamic navigation model')
parser.add_option("--usePPP", action='store_true', default=None, help='enable precise point positioning')
parser.add_option("--dots", action='store_true', default=False, help='print a dot on each message')


(opts, args) = parser.parse_args()

dev = ublox.UBlox(opts.port, baudrate=opts.baudrate, timeout=2)
dev.set_logfile(opts.log, append=opts.append)
dev.set_binary()
dev.configure_poll_port()
dev.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)
#dev.configure_poll(ublox.CLASS_MON, ublox.MSG_MON_HW)
dev.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
dev.configure_port(port=ublox.PORT_USB, inMask=1, outMask=1)
dev.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
dev.configure_poll_port()
dev.configure_poll_port(ublox.PORT_SERIAL1)
dev.configure_poll_port(ublox.PORT_SERIAL2)
dev.configure_poll_port(ublox.PORT_USB)
dev.configure_solution_rate(rate_ms=1000)

dev.set_preferred_dynamic_model(opts.dynModel)
dev.set_preferred_usePPP(opts.usePPP)

dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_STATUS, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SOL, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELECEF, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 1)
dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 1)
dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 1)
dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SVSI, 1)
dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_ALM, 1)
dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_EPH, 1)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_TIMEGPS, 5)
dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_CLOCK, 5)
#dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS, 5)

from ephemeris import EphemerisData
import satPosition
from satelliteData import rawPseudoRange
from satelliteData import SatelliteData

eph = [EphemerisData() for x in range(32)]
while True:
    msg = dev.receive_message()
    if msg is None:
        if opts.reopen:
            dev.close()
            dev = ublox.UBlox(opts.port, baudrate=opts.baudrate, timeout=2)
            dev.set_logfile(opts.log, append=True)
            sys.stdout.write('R')
            continue
        break
    if opts.show:
        print(str(msg))

        #SFRBX handler
        if msg.name() == 'RXM_SFRBX':
            svid = msg._fields['svid']

            eph[svid-1].fill_ephemeris(msg)
            #print(svid)
            if eph[svid-1].is_filled():
                pass


        #RAWX handler
        if msg.name() == 'RXM_RAWX':
            gps_week = msg._fields['week']
            time_of_week = msg._fields['rcvTow']
            rawPR = rawPseudoRange(gps_week, time_of_week)

            for mes in msg._recs:
                svid    = mes['svId']
                cpMes   = mes['cpMes']
                prMes   = mes['prMes']
                cno     = mes['cno']
                trkStat = mes['trkStat']
                #rawPR.add(svid, prMes, cpMes, 0, trkStat, cno)

        sys.stdout.flush()

    elif opts.dots:
        sys.stdout.write('.')
        sys.stdout.flush()
