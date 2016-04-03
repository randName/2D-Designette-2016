import urllib2

def get_targets( url, challenge=False ):
    """Parse targets from url"""

    req = urllib2.Request( url, headers={'User-Agent':'Mozilla/5.0'} )
    data = urllib2.urlopen( req )

    if not data: return None

    path = 'X'
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

    return path

if __name__ == "__main__":

    urlsz = 'http://people.sutd.edu.sg/~oka_kurniawan/10_009/y2015/2d/tests/level%s_%s.inp'

    for i in (1,2):
        for j in (1,2,3):
            print "level%s_%s.inp" % (i,j)
            print "Path:", ' -> '.join([ p for p in get_targets( urlsz % (i,j) ) ])