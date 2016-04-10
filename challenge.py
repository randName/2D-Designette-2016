import urllib2
from time import strftime

def get_targets( url ):
    """Parse targets from url"""

    req = urllib2.Request( url, headers={'User-Agent':'Mozilla/5.0'} )
    data = urllib2.urlopen( req )

    if not data: return '', []

    path = 'X'
    all_trips = []
    targets = {}

    for line in data:
        lz = line.split()

        plates = int(lz[1])
        trips = plates/6
        tt = ( lz[0]*6, )*trips
        leftovers = plates % 6
        if leftovers:
            trips += 1
            tt += ( lz[0]*leftovers, )
        path += (lz[0]+'X')*trips
        all_trips += tt

    return path, all_trips

def logto( logfile=None, cloud=None ):
    def logto_decorator( func ):
        def func_wrapper( data ):
            tolog = func( data )
            if isinstance( tolog, dict ) and cloud:
                location = tolog.pop('location')
                if location != 'X':
                    cloud.put( '/2d/', ( 'station%s' % location ), tolog )
            elif isinstance( tolog, basestring ) and logfile:
                with open( logfile, 'a' ) as f:
                    f.write( tolog + "\n" )
            return tolog
        return func_wrapper
    return logto_decorator

def log( data ):
    if isinstance( data, dict ):
        data['time'] = strftime("%H:%M:%S|%d/%m/%y")
    else:
        logstr = strftime("<%H:%M:%S> || <%d-%m-%Y> || ")
        path = list(data)
        location = path.pop(0)
        if not path:
            logstr += "Finished, arrived at %s" % location
        elif location == 'X':
            logstr += "Collect Plates at X"
        else:
            logstr += "Expose Plates at %s" % location
        data = logstr
    return data

if __name__ == "__main__":

    urlsz = 'http://people.sutd.edu.sg/~oka_kurniawan/10_009/y2015/2d/tests/level%s_%s.inp'

    for i in (1,2):
        for j in (1,2,3):
            print "level%s_%s.inp" % (i,j)
            path, trips = get_targets( urlsz % (i,j) )
            print "Path:", ' -> '.join( path )
