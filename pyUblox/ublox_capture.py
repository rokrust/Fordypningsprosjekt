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
from util import speedOfLight
import scipy.io

satData = SatelliteData()
data = {    'pseudorange'   : [],
            'satPos'        : [],
            't'             : [],
            'ionospheric'   : [],
            'relativistic'  : [],
            'sv_clock'      : []
        }
zeroRow = [0 for x in range(32)]
zeroPos = [[0, 0, 0] for x in range(32)]
i = 0
while True:
    msg = dev.receive_message()
    # end of file
    if msg is None:
        if opts.reopen:
            dev.close()
            dev = ublox.UBlox(opts.port, baudrate=opts.baudrate, timeout=2)
            dev.set_logfile(opts.log, append=True)
            sys.stdout.write('R')
            continue

        scipy.io.savemat('Satellite_data_base', data, )
        print "Parsing done.. saved data in Sattelite_data.mat"
        break

    msg.unpack()
    satData.add_message(msg)

    if msg.name() == 'RXM_RAWX':                                #New pseudorange measurement
            if len(satData.locked_satellites) >= 4:             #At least four satellites locked

                data['pseudorange'].append(list(zeroRow))
                data['satPos'].append(list(zeroPos))
                data['t'].append(list(zeroRow))
                data['relativistic'].append(list(zeroRow))
                data['sv_clock'].append(list(zeroRow))

                for svid in satData.locked_satellites:
                    pr = satData.raw.prMeasured[svid]

                    t_flight = pr/speedOfLight                  #time from transmission to receive time
                    t_sv = satData.raw.time_of_week - t_flight  #transmission time

                    #Satellite position estimate
                    satPosition(satData, svid, t_sv)
                    correctPosition(satData, svid, t_flight)
                    pos = satData.satpos[svid]

                    #Add to mat-file
                    data['satPos'][-1][svid-1] = list([pos.X, pos.Y, pos.Z])
                    data['pseudorange'][-1][svid - 1] = satData.raw.prMeasured[svid] + sum(pos.extra)*speedOfLight
                    data['t'][-1][svid - 1] = t_sv
                    data['sv_clock'][-1][svid - 1] = pos.extra[0]
                    data['relativistic'][-1][svid - 1] = pos.extra[1]

                    ion = satData.ephemeris[svid].ionospheric
                    if ion != None and ion.valid:
                        ionospheric = [ion.a0, ion.a1, ion.a2, ion.a3, ion.b0, ion.b1, ion.b2, ion.b3]
                        data['ionospheric'] = ionospheric
                pass

    if opts.show:
        print(str(msg))
        sys.stdout.flush()

    elif opts.dots:
        sys.stdout.write('.')
        sys.stdout.flush()