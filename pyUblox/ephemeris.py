import util

class EphemerisData:
    '''container for parsing a AID_EPH message
    Thanks to Sylvain Munaut <tnt@246tNt.com>
    http://cgit.osmocom.org/cgit/osmocom-lcs/tree/gps.c
    for the C version of this parser

    See IS-GPS-200F.pdf Table 20-III for the field meanings, scaling factors and
    field widths
    '''

    def GET_FIELD_U(self, w, nb, pos):
        return (((w) >> (pos)) & ((1<<(nb))-1))

    def twos_complement(self, v, nb):
        sign = v >> (nb-1)
        value = v
        if sign != 0:
            value = value - (1<<nb)
        return value

    def GET_FIELD_S(self, w, nb, pos):
        v = self.GET_FIELD_U(w, nb, pos)
        return self.twos_complement(v, nb)

    def __init__(self, msg):
        from math import pow
        if not msg.have_field('numWords'):
            # it doesn't contain the optional part
            self.valid = False
            return

        #bit 20, 21, 22 of HOW contains subframe id.
        subframe_id = self.GET_FIELD_U(msg._recs[1]['dwrd'], 3, 8)

        dwrds = [];
        for i in range(2, 10):
            #drop parity bits
            dwrds.append(msg._recs[i]['dwrd'] >> 6)

        self._msg = msg
        self.svid = msg.svid

        #Check if ephemeris is filled
        self.subframe1_valid = False
        self.subframe2_valid = False
        self.subframe3_valid = False

        # Definition of Pi used in the GPS coordinate system
        gpsPi          = 3.1415926535898

        if subframe_id == 1:
            pass

        elif subframe_id == 2:
            pass

        elif subframe_id == 3:
            pass

        self.valid = (self.iode1 == self.iode2) and (self.iode1 == (self.iodc & 0xff))
        self.iode = self.iode1
        self.ephemeris_filled = self.subframe1_valid and self.subframe2_valid and self.subframe3_valid

    def subframe1(self, dwrds):
        week_no = self.GET_FIELD_U(dwrds[0], 10, 14)
        code_on_l2 = self.GET_FIELD_U(dwrds[0], 2, 12)
        sv_ura = self.GET_FIELD_U(dwrds[0], 4, 8)
        sv_health = self.GET_FIELD_U(dwrds[0], 6, 2 )
        l2_p_flag = self.GET_FIELD_U(dwrds[1], 1, 23)
        t_gd = self.GET_FIELD_S(dwrds[4], 8, 0)

        t_oc = self.GET_FIELD_U(dwrds[5], 16, 0)
        a_f2 = self.GET_FIELD_S(dwrds[6], 8, 16)
        a_f1 = self.GET_FIELD_S(dwrds[6], 16, 0)
        a_f0 = self.GET_FIELD_S(dwrds[7], 22, 2)

        self._rsvd1 = self.GET_FIELD_U(dwrds[1], 23, 0)
        self._rsvd2 = self.GET_FIELD_U(dwrds[2], 24, 0)
        self._rsvd3 = self.GET_FIELD_U(dwrds[3], 24, 0)
        self._rsvd4 = self.GET_FIELD_U(dwrds[4], 16, 8)
        self.iodc = (self.GET_FIELD_U(dwrds[0], 2, 0) << 8) | self.GET_FIELD_U(msg.dwrds[5], 8, 16)

        self.Tgd = t_gd * pow(2, -31)

        # clock correction information
        self.toc = t_oc * pow(2, 4)
        self.af0 = a_f0 * pow(2, -31)
        self.af1 = a_f1 * pow(2, -43)
        self.af2 = a_f2 * pow(2, -55)

        self.subframe1_valid = True

    def subframe2(self, dwrds):
        c_rs = self.GET_FIELD_S(dwrds[0], 16, 0)
        delta_n = self.GET_FIELD_S(dwrds[1], 16, 8)
        m_0 = (self.GET_FIELD_S(dwrds[1], 8, 0) << 24) | self.GET_FIELD_U(msg.dwrds[2], 24, 0)
        c_uc = self.GET_FIELD_S(dwrds[3], 16, 8)
        e = (self.GET_FIELD_U(dwrds[3], 8, 0) << 24) | self.GET_FIELD_U(msg.dwrds[4], 24, 0)
        c_us = self.GET_FIELD_S(dwrds[5], 16, 8)
        a_powhalf = (self.GET_FIELD_U(dwrds[5], 8, 0) << 24) | self.GET_FIELD_U(msg.dwrds[6], 24, 0)
        t_oe = self.GET_FIELD_U(dwrds[7], 16, 8)
        fit_flag = self.GET_FIELD_U(dwrds[7], 1, 7)
        self.aodo = self.GET_FIELD_U(dwrds[7], 5, 2)

        self.crs = c_rs * pow(2, -5)
        self.cuc = c_uc * pow(2, -29)
        self.cus = c_us * pow(2, -29)
        self.A = pow(a_powhalf * pow(2, -19), 2.0)
        self.deltaN = delta_n * pow(2, -43) * gpsPi
        self.ecc = e * pow(2, -33)
        self.M0 = m_0 * pow(2, -31) * gpsPi
        self.toe = t_oe * pow(2, 4)
        self.iode1 = self.GET_FIELD_U(msg.dwrds[0], 8, 16)

        self.subframe2_valid = True

    def subframe3(self, dwrds):
        c_ic = self.GET_FIELD_S(dwrds[0], 16, 8)
        omega_0 = (self.GET_FIELD_S(dwrds[0], 8, 0) << 24) | self.GET_FIELD_U(msg.dwrds[1], 24, 0)
        c_is = self.GET_FIELD_S(dwrds[2], 16, 8)
        i_0 = (self.GET_FIELD_S(dwrds[2], 8, 0) << 24) | self.GET_FIELD_U(msg.dwrds[3], 24, 0)
        c_rc = self.GET_FIELD_S(dwrds[4], 16, 8)
        w = (self.GET_FIELD_S(dwrds[4], 8, 0) << 24) | self.GET_FIELD_U(msg.dwrds[5], 24, 0)
        omega_dot = self.GET_FIELD_S(dwrds[6], 24, 0)
        idot = self.GET_FIELD_S(dwrds[7], 14, 2)

        self.cis = c_is * pow(2, -29)
        self.cic = c_ic * pow(2, -29)
        self.crc = c_rc * pow(2, -5)
        self.i0 = i_0 * pow(2, -31) * gpsPi
        self.idot = idot * pow(2, -43) * gpsPi
        self.omega = w * pow(2, -31) * gpsPi
        self.omega_dot = omega_dot * pow(2, -43) * gpsPi
        self.omega0 = omega_0 * pow(2, -31) * gpsPi
        self.iode2 = self.GET_FIELD_U(msg.dwrds[7], 8, 16)

        self.subframe3_valid = True


    def __eq__(self, other):
        '''allow for equality testing
        See http://stackoverflow.com/questions/3550336/comparing-two-objects
        '''
        attrs = [ 'svid', 'valid', 'Tgd', 'A', 'cic', 'cis', 'crc', 'crs', 'cuc', 'cus',
                  'deltaN', 'ecc', 'i0', 'idot', 'M0', 'omega', 'omega_dot', 'omega0',
                  'toe', 'toc', 'af0', 'af1', 'af2', 'iode' ]
        for a in attrs:
            v1, v2 = [getattr(obj, a, None) for obj in [self, other]]
            if v1 is None or v2 is None or v1 != v2:
                return False
        return True

    def __ne__(self, other):
        '''allow for equality testing'''
        return not self.__eq__(other)


class IonosphericData:
    '''decode ionospheric data from a RXM_SFRB subframe 4 message
    see http://home-2.worldonline.nl/~samsvl/nav2eu.htm
    '''

    def extract_uint8(self, v, b):
        return (v >> (8*(3-b))) & 255

    def extract_int8(self, v, b):
        value = self.extract_uint8(v, b)
        if value > 127:
            value -= 256
        return value
        
    def __init__(self, msg):
        '''parse assuming a subframe 4 page 18 message containing ionospheric data'''
        words = msg.dwrd
        for i in range(10):
            words[i] = (words[i] & 0xffffff)
        words[0] &= 0xff0000
        if not words[0] in [0x8b0000, 0x740000]:
            #print("words[0]=0x%06x" % words[0])
            self.valid = False
            return
        if words[0] == 0x740000:
            print("SFRB invert")
            for i in range(10):
                words[i] ^= 0xffffff

        self.svid = msg.svid
        self.id = (words[1] >> 2) & 0x07
        self.pageID = (words[2] & 0x3f0000) >> 16

        self.a0     = self.extract_int8(words[2], 2) * pow(2, -30)
        self.a1     = self.extract_int8(words[2], 3) * pow(2, -27)
        self.a2     = self.extract_int8(words[3], 1) * pow(2, -24)
        self.a3     = self.extract_int8(words[3], 2) * pow(2, -24)
        self.b0     = self.extract_int8(words[3], 3) * pow(2, 11)
        self.b1     = self.extract_int8(words[4], 1) * pow(2, 14)
        self.b2     = self.extract_int8(words[4], 2) * pow(2, 16)
        self.b3     = self.extract_int8(words[4], 3) * pow(2, 16)
        self.leap   = self.extract_uint8(words[8], 1)

        # this checks if we have the right subframe
        self.valid  = (self.pageID == 56 and self.id == 4)
        #print("svid=%u id=%u pageID=%u" % (self.svid, self.id, self.pageID))
#        '''
        if self.valid:
            print("a0=%g a1=%g a2=%g a3=%g b0=%g b1=%g b2=%g b3=%g leap=%u" % (
                self.a0, self.a1, self.a2, self.a3,
                self.b0, self.b1, self.b2, self.b3,
                self.leap))
#                '''
                  
    def __eq__(self, other):
        '''allow for equality testing
        See http://stackoverflow.com/questions/3550336/comparing-two-objects
        '''
        attrs = [ 'svid', 'valid', 'a0', 'a1', 'a2', 'a3',
                  'b0', 'b1', 'b2', 'b3', 'leap', 'id', 'pageID' ]
        for a in attrs:
            v1, v2 = [getattr(obj, a, None) for obj in [self, other]]
            if v1 is None or v2 is None or v1 != v2:
                return False
        return True

    def __ne__(self, other):
        '''allow for equality testing'''
        return not self.__eq__(other)