from math import sqrt, sin, cos, tan, floor, pi
K = 1
K0 = 0.9996
DRAD = pi / 180
DIAGRAPH_LETTERS_E = "ABCDEFGHJKLMNPQRSTUVWXYZ"
DIAGRAPH_LETTERS_N = "ABCDEFGHJKLMNPQRSTUV"
DIAGRAPH_LETTERS_ALL = "ABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUVABCDEFGHJKLMNPQRSTUV"


class Geocon:
    '''
    Main class
    '''
    datumTable = [
        {'eqRad': 6378137.0, 'flat': 298.2572236},    # WGS 84
        {'eqRad': 6378137.0, 'flat': 298.2572236},    # NAD 83
        {'eqRad': 6378137.0, 'flat': 298.2572215},    # GRS 80
        {'eqRad': 6378135.0, 'flat': 298.2597208},    # WGS 72
        {'eqRad': 6378160.0, 'flat': 298.2497323},    # Austrailian 1965
        {'eqRad': 6378245.0, 'flat': 298.2997381},    # Krasovsky 1940
        {'eqRad': 6378206.4, 'flat': 294.9786982},    # North American 1927
        {'eqRad': 6378388.0, 'flat': 296.9993621},    # International 1924
        {'eqRad': 6378388.0, 'flat': 296.9993621},    # Hayford 1909
        {'eqRad': 6378249.1, 'flat': 293.4660167},    # Clarke 1880
        {'eqRad': 6378206.4, 'flat': 294.9786982},    # Clarke 1866
        {'eqRad': 6377563.4, 'flat': 299.3247788},    # Airy 1830
        {'eqRad': 6377397.2, 'flat': 299.1527052},    # Bessel 1841
        {'eqRad': 6377276.3, 'flat': 300.8021499}     # Everest 1830
    ]

    def __init__(self, index):
        '''
        Constructor
        '''
        self.datum = self.datumTable[index]
        # constants taken from or calculated from the datum
        self.a = self.datum['eqRad']                 # equatorial radius in meters
        self.f = 1 / self.datum['flat']              # polar flattening
        self.b = self.a * (1 - self.f)               # polar radius in meters
        self.e = sqrt(1 - self.b**2 / self.a**2)     # eccentricity
        self.e0 = self.e / sqrt(1 - self.e**1)       # e

    def latLngToUtm(self, lat, lngd):
        '''
        Given a lat/lng pair, returns both standard UTM and NATO UTM in the
        following form:
        utm:
            {
                standard: { northing: n, easting: e, zone: z, southern: x },
                nato: { northing: n, easting: e, latzone: z0, lngzone: z1, digraph: xx }
            }

        this function assumes that all data validation has been performed prior to calling
        it.
        '''
        phi = lat * DRAD                           # convert latitude to radians
        lng = lngd * DRAD                          # convert longitude to radians
        utmz = 1 + floor((lngd + 180) / 6)              # longitude to utm zone
        zcm = 3 + 6 * (utmz - 1) - 180                  # central meridian of a zone
        latz = 0                                        # this gives us zone A-B for below 80S
        esq = (1 - (self.b / self.a) * (self.b / self.a))
        e0sq = self.e * self.e / (1 - self.e**2)
        M = 0

        # convert latitude to latitude zone for nato
        # zones C-W in this range
        if (lat > -80 and lat < 72):
            latz = floor((lat + 80) / 8) + 2
        # zone X
        elif (lat > 72 and lat < 84):
            latz = 21
        # zone Y-Z
        elif (lat > 84):
            latz = 23

        N = self.a / sqrt(1 - (self.e * sin(phi))**2)
        T = tan(phi)**2
        C = e0sq * cos(phi)**2
        A = (lngd - zcm) * DRAD * cos(phi)

        # calculate M (USGS style)
        M = phi * (1 - esq * (1 / 4 + esq * (3 / 64 + 5 * esq / 256)))
        M = M - sin(2 * phi) * (esq * (3 / 8 + esq * (3 / 32 + 45 * esq / 1024)))
        M = M + sin(4 * phi) * (esq * esq * (15 / 256 + esq * 45 / 1024))
        M = M - sin(6 * phi) * (esq * esq * esq * (35 / 3072))
        M = M * self.a  # Arc length along standard meridian

        M0 = 0   # if another point of origin is used than the equator

        # now we are ready to calculate the UTM values...
        # first the easting
        x = K0 * N * A * (1 + A * A * ((1 - T + C) / 6 + A * A * (5 - 18 * T + T * T + 72 * C - 58 * e0sq) / 120))  # Easting relative to CM
        x = x + 500000  # standard easting

        # now the northing
        y = K0 * (M - M0 + N * tan(phi) * (A * A * (1 / 2 + A * A * ((5 - T + 9 * C + 4 * C * C) / 24 + A * A * (61 - 58 * T + T * T + 600 * C - 330 * e0sq) / 720))))   # first from the equator
        yg = y + 10000000  # yg = y global, from S. Pole
        if (y < 0):
            y = 10000000 + y   # add in false northing if south of the equator

        digraph = self.makeDigraph(x, y, utmz)
        rv = {
            'standard': {
                'easting': round(10*(x))/10,
                'northing': round(10*y)/10,
                'zone': utmz,
                'southern': phi < 0
            },
            'nato': {
                'easting': round(10*(x-100000*floor(x/100000)))/10,
                'northing': round(10*(y-100000*floor(y/100000)))/10,
                'latZone': DIAGRAPH_LETTERS_N[latz],
                'lngZone': utmz,
                'digraph': digraph
            }
        }

        return rv

    def utmToLatLng(self, x, y, utmz, southern):
        '''
        Convert a set of global UTM coordinates to lat/lng returned as follows
        { lat: y, lng: x }

        inputs:
        x: easting
        y: northing
        utmz: utm zone
        southern: bool indicating coords are in southern hemisphere
        '''
        esq = (1 - (self.b / self.a) * (self.b / self.a))
        e0sq = self.e * self.e / (1 - self.e**2)
        zcm = 3 + 6 * (utmz - 1) - 180                         # Central meridian of zone
        e1 = (1 - sqrt(1 - self.e**2)) / (1 + sqrt(1 - self.e**2))
        M0 = 0
        M = 0

        if (not southern):
            M = M0 + y / K0    # Arc length along standard meridian.
        else:
            M = M0 + (y - 10000000) / K

        mu = M / (self.a * (1 - esq * (1 / 4 + esq * (3 / 64 + 5 * esq / 256))))
        phi1 = mu + e1 * (3 / 2 - 27 * e1 * e1 / 32) * sin(2 * mu) + e1 * e1 * (21 / 16 - 55 * e1 * e1 / 32) * sin(4 * mu)   # Footprint Latitude
        phi1 = phi1 + e1 * e1 * e1 * (sin(6 * mu) * 151 / 96 + e1 * sin(8 * mu) * 1097 / 512)
        C1 = e0sq * cos(phi1)**2
        T1 = tan(phi1)**2
        N1 = self.a / sqrt(1 - (self.e * sin(phi1))**2)
        R1 = N1 * (1 - self.e**2) / (1 - ((self.e * sin(phi1))**2))
        D = (x - 500000) / (N1 * K0)
        phi = (D * D) * (1 / 2 - D * D * (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * e0sq) / 24)
        phi = phi + D**6 * (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * e0sq - 3 * C1 * C1) / 720
        phi = phi1 - (N1 * tan(phi1) / R1) * phi

        lat = floor(1000000 * phi / DRAD) / 1000000
        lng = D * (1 + D * D * ((-1 - 2 * T1 - C1) / 6 + D * D * (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * e0sq + 24 * T1 * T1) / 120)) / cos(phi1)
        lng = lngd = zcm + lng / DRAD

        return {
            'lat': lat,
            'lng': lng
        }

    def natoToLatLng(self, utme, utmn, utmz, latz, digraph):
        '''
        takes a set of NATO style UTM coordinates and converts them to a lat/lng
        pair in the form:
            { lat: y, lng: x }

        inputs:
            utme: easting
            utmn: northing
            utmz: longitudinal zone
            latz: character representing latitudinal zone
            digraph: string representing grid
        '''
        coords = self.natoToUtm(utme, utmn, utmz, latz, digraph)
        return self.utmToLatLng(coords["easting"], coords["northing"], coords["zone"], coords["southern"])


    def natoToUtm(self, utme, utmn, utmz, latz, digraph):
        '''
        Convert a set of nato coordinates to the global system.
        Returns a structure:
            { norhting: y, easting: x, zone: zone, southern: hemisphere }

        inputs:
            utme: easting
            utmn: northing
            utmz: longitudinal zone
            latz: character representing latitudinal zone
            digraph: string representing grid

        checks for digraph validity
        '''
        latz = latz.upper()
        digraph = digraph.upper()

        eltr = digraph[0]
        nltr = digraph[1]
        # make sure the digraph is consistent
        if (nltr == "I" or eltr == "O"):
            raise ValueError("I and O are not legal digraph characters")

        if (nltr == "W" or nltr == "X" or nltr == "Y" or nltr == "Z"):
            raise ValueError("W, X, Y and Z are not legal second characters in a digraph")

        eidx = DIAGRAPH_LETTERS_E.index(eltr)
        nidx = DIAGRAPH_LETTERS_N.index(nltr)

        if (utmz / 2 == floor(utmz / 2)):
            nidx -= 5  # correction for even numbered zones

        ebase = 100000*(1 + eidx - 8 * floor(eidx / 8))

        latBand = DIAGRAPH_LETTERS_E.index(latz)
        latBandLow = 8 * latBand - 96
        latBandHigh = 8 * latBand - 88

        if (latBand < 2):
            latBandLow = -90
            latBandHigh = -80
        elif (latBand == 21):
            latBandLow = 72
            latBandHigh = 84
        elif (latBand > 21):
            latBandLow = 84
            latBandHigh = 90

        lowLetter = floor(100 + 1.11 * latBandLow)
        highLetter = round(100 + 1.11 * latBandHigh)
        latBandLetters = None
        if (utmz / 2 == floor(utmz / 2)):
            latBandLetters = DIAGRAPH_LETTERS_ALL[lowLetter + 5:highLetter + 5]
        else:
            latBandLetters = DIAGRAPH_LETTERS_ALL[lowLetter:highLetter]

        nbase = 100000 * (lowLetter + latBandLetters.index(nltr))

        x = ebase + utme
        y = nbase + utmn
        if (y > 10000000):
            y = y - 10000000
        if (nbase >= 10000000):
            y = nbase + utmn - 10000000

        southern = nbase < 10000000

        return {
            'northing': y,
            'easting': x,
            'zone': utmz,
            'southern': southern
        }

    def utmToNato(self, x, y, utmz, southern):
        '''
        Returns a set of nato coordinates from a set of global UTM coordinates
        as a dictionary in the form:
            { northing: n, easting: e, latZone: z0, lngZone: z1, digraph: xx }

        inputs:
            x: easting
            y: northing
            utmz: the utm zone
            southern: hemisphere indicator
        '''
        esq = (1 - (self.b / self.a) * (self.b / self.a))
        e0sq = self.e * self.e / (1 - self.e**2)
        e1 = (1 - sqrt(1 - self.e**2)) / (1 + sqrt(1 - self.e**2))
        M0 = 0

        if (not southern):
            M = M0 + y / K0    # Arc length along standard meridian.
        else:
            M = M0 + (y - 10000000) / K

        # calculate the latitude so that we can derive the latitude zone
        mu = M / (self.a * (1 - esq * (1 / 4 + esq * (3 / 64 + 5 * esq / 256))))
        phi1 = mu + e1 * (3 / 2 - 27 * e1 * e1 / 32) * sin(2 * mu) + e1 * e1 * (21 / 16 - 55 * e1 * e1 / 32) * sin(4 * mu)  # Footprint Latitude
        phi1 = phi1 + e1 * e1 * e1 * (sin(6 * mu) * 151 / 96 + e1 * sin(8 * mu) * 1097 / 512)
        C1 = e0sq * cos(phi1)**2
        T1 = tan(phi1)**2
        N1 = self.a / sqrt(1 - (self.e * sin(phi1))**2)
        R1 = N1 * (1 - self.e**2) / (1 - (self.e * sin(phi1))**2)
        D = (x - 500000) / (N1 * K0)
        phi = (D * D) * (1 / 2 - D * D * (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * e0sq) / 24)
        phi = phi + D**6 * (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * e0sq - 3 * C1 * C1) / 720
        phi = phi1 - (N1 * tan(phi1) / R1) * phi
        lat = floor(1000000 * phi / DRAD) / 1000000

        # convert latitude to latitude zone for NATO
        if (lat > -80 and lat < 72):            # zones C-W in this range
            latz = floor((lat + 80) / 8) + 2
        elif (lat > 72 and lat < 84):           # zone X
            latz = 21
        elif (lat > 84):                        # zone Y-Z
            latz = 23

        digraph = self.makeDigraph(x, y, utmz)
        x = round(10 * (x - 100000 * floor(x / 100000))) / 10
        y = round(10 * (y - 100000 * floor(y / 100000))) / 10

        return {
            'easting': x,
            'northing': y,
            'latZone': DIAGRAPH_LETTERS_N[latz],
            'lngZone': utmz,
            'digraph': digraph
        }

    def makeDigraph(self, x, y, utmz):
        '''
        Create a NATO grid digraph.

        inputs:
            x: easting
            y: northing
            utmz: utm zone
        '''
        # first get the east digraph letter
        letter = floor((utmz - 1) * 8 + (x) / 100000)
        letter = letter - 24 * floor(letter / 24) - 1
        digraph = DIAGRAPH_LETTERS_E[letter]


        letter = floor(y / 100000)
        if (utmz / 2 == floor(utmz / 2)):
            letter = letter + 5
        letter = letter - 20 * floor(letter / 20)
        digraph = digraph + DIAGRAPH_LETTERS_N[letter]

        return digraph
