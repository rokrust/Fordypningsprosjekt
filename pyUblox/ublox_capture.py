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


from satPosition import satPosition
from satPosition import correctPosition
from satelliteData import SatelliteData
from util import PosVector
from util import speedOfLight
import scipy.io

import rangeCorrection as RC

satData = SatelliteData()
data = {    'pseudorange' : [],
            'satPos' : []}
zeroRow = [0 for x in range(32)]
zeroPos = [[0, 0, 0] for x in range(32)]
visible_satellites = []

while True:
    msg = dev.receive_message()

    msg.unpack()


    satData.add_message(msg)

    if msg.name() == 'RXM_RAWX' and satData.valid():
            data['pseudorange'].append(list(zeroRow))
            data['satPos'].append(list(zeroPos))

            for prData in msg._recs:
                svid = prData['svId']
                t_flight = prData['prMes']/speedOfLight
                t_sv = msg._fields['rcvTow'] - t_flight

                #Satellite position estimate
                satPosition(satData, svid, t_sv)
                correctPosition(satData, svid, t_flight)
                pos = satData.satpos[svid]

                #Corrections
                dRC_SV  = RC.sv_clock_correction(satData, svid, t_sv, pos.extra)
                #dRC_TRP = RC.tropospheric_correction_standard(satData, svid) requires elevation which requires estimated position
                dRC_ION = 0
                if not satData.ionosperic == None:
                    dRC_ION = RC.ionospheric_correction(satData, svid, t_sv, pos)

                satData.raw.prMeasured[svid] += -dRC_SV + dRC_ION# + dRC_TRP

                #Add to mat-file
                data['satPos'][-1][svid-1] = list([pos.X, pos.Y, pos.Z])
                data['pseudorange'][-1][svid - 1] = satData.raw.prMeasured[svid]

            ## Corrections

    #end of file
    if msg is None:
        if opts.reopen:
            dev.close()
            dev = ublox.UBlox(opts.port, baudrate=opts.baudrate, timeout=2)
            dev.set_logfile(opts.log, append=True)
            sys.stdout.write('R')
            continue

        scipy.io.savemat('Satellite_data', data, )
        print "Parsing done.. saved data in Sattelite_data.mat"
        break
    if opts.show:
        print(str(msg))
        sys.stdout.flush()

    elif opts.dots:
        sys.stdout.write('.')
        sys.stdout.flush()