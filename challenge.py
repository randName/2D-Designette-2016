import urllib2

def get_targets( url, challenge=False ):
    """Parse targets from url"""

    req = urllib2.Request( url, headers={'User-Agent':'Mozilla/5.0'} )
    data = urllib2.urlopen( req )

    if not data: return None, None

    path = 'X'
    trips = None
    targets = {}

    for line in data:
        lz = line.split()

        if challenge:
            targets[lz[0]] = int(lz[1])
        else:
            plates = int(lz[1])
            trips = plates/6
            if plates % 6: trips += 1
            path += (lz[0]+'X')*trips

    if challenge:
        trips, path = decide_path( targets )

    return path, trips

def logto( logfile=None, cloud=None ):
    def logto_decorator( func ):
        def func_wrapper( data ):
            tolog = func( data )
            if isinstance( tolog, dict ) and cloud:
                location = tolog.pop('location')
                cloud.put( '/2d/', ( 'station%s' % location ), tolog )
            elif isinstance( tolog, basestring ) and logfile:
                logfile.write( tolog )
            return tolog
        return func_wrapper
    return logto_decorator

def log( data ):
    if isinstance( data, dict ):
        data['time'] = time.strftime("%H:%M:%S|%d/%m/%y")
    else:
        logstr = time.strftime("<%H:%M:%S> || <%d-%m-%Y> || ")
        location = list(data).pop(0)
        if not data:
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
            print "Path:", ' -> '.join([ p[0] for p in get_targets( urlsz % (i,j) ) ])
