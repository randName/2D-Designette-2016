from itertools import combinations, permutations

class Map():

    paths_0 = { 'XA': "R", 'XB': "F", 'XC': "L" }

    paths_1 = {
        'AB': "R", 'AC': "FR", 'AD': "FF", 'AX': "LL", 'AH': "FLR",
        'BC': "RR", 'BD': "RF", 'BX': "FL", 'BH': "FRF", 'CD': "R",
        'CX': "FLF", 'CH': "FR", 'DX': "FRL", 'DH': "RR", 'HX': "FF"
    }

    costs = {
        'AB': 5, 'AC': 8, 'AD': 7, 'AX': 9, 'AH': 11,
        'BC': 9, 'BD': 8, 'BX': 8, 'BH': 10, 'CD': 5,
        'CX': 10, 'CH': 8, 'DX': 11, 'DH': 9,
    }
    
    def __init__( s, level=0 ):
        s.level = level

    def getpath( s, start, end ):
        pa = s.paths_1 if s.level else s.paths_0
        if start + end in pa:
            return pa[ start+end ]
        if end + start in pa:
            p = pa[ end+start ][::-1]
            return p.replace('R','Z').replace('L','R').replace('Z','L')
        return ''

    def getcost( s, start, end ):
        if s.level == 0: return 1
        if start + end in s.costs:
            return s.costs[ start+end ]
        if end + start in s.costs:
            return s.costs[ end+start ]
        return 0

    def walkpath( s, r ):
        return sum( ( ( s.getpath(r[i],r[i+1]), r[i+1] ) for i in xrange(len(r)-1) ), () )

    def walkcost( s, r ):
        return sum( s.getcost(r[i],r[i+1])+1 for i in xrange(len(r)-1) )

def sstr( s ): return ''.join( sorted( s ) )

def get_path( trips ):
    return ''.join( sstr(set(t)) + 'X' for t in trips )[:-1]

def int_partitions(n):
    if n == 0:
        yield ()
        return

    for p in int_partitions(n-1):
        yield (1,) + p
        if p and ( len(p) < 2 or p[1] > p[0] ):
            yield ( p[0] + 1, ) + p[1:]

def get_partitions(n, min_len=1, max_len=None, max_partitions=None ):
    if max_len == None: max_len = n

    for p in int_partitions(n):
        if max_partitions and len(p) > max_partitions:
            continue
        if len(p) > 1 and min(p[1:]) < min_len:
            continue
        if p and max(p) > max_len:
            continue
        yield p

def get_combinations( s, partition ):
    ss = sstr(s)

    if len( partition ) <= 1:
        yield (ss,)
        return

    previous = tuple()
    for c in combinations( ss, partition[0] ):
        if c == previous: continue
        previous = c
        sc = ''.join(c)
        news = ss
        for r in set(sc): news = news.replace(r,'',sc.count(r))
        for nc in get_combinations( news, partition[1:] ):
            yield tuple( sorted( (sc,) + nc ) )

def decide_path( targets ):
    """Takes in a dict of targets and decides the path to take."""

    def submit( path, trips, sortl=True ):
        if sortl: trips.sort()
        path[1] = get_path( trips )
        return trips, ''.join( path )

    trips = []
    path = [ 'X', '', 'H' ]

    for k,v in targets.iteritems():
        if v >= 6:
            trips.extend( [ k*6 ]*(v/6) )
            v = v%6
            targets[k] = v

    n_pl = sum( targets.itervalues() )

    if trips and not n_pl:
        return submit( path, trips )

    if n_pl%6 and sum( 1 for v in targets.itervalues() if v ) <= 1+n_pl/6:
        trips.extend( k*v for k,v in targets.iteritems() )
        return submit( path, trips )

    head_trips = len( trips )

    def shunt( p ):
        targets[p[0]] -= p[1]
        return p[0]*p[1]

    def transfer( t ):
        try:
            pt = sstr( t.keys() )
            pl = sstr( shunt( p ) for p in t.iteritems() )
        except AttributeError:
            pt = t
            pl = shunt( ( t, targets[t] ) )
        trips.insert( head_trips, pl )
        return pt

    def addon( tg, head=False ):
        if head:
            path[0] += transfer( tg ) + 'X'
            return 1
        else:
            path[2] = 'X' + transfer( tg ) + path[2]

    if n_pl == 18:
        if targets['A'] == 5:
            head_trips += addon( { 'A': 5, 'B': 1 }, True )
        if targets['D'] == 5:
            addon( { 'D': 5, 'B': 1 } )
        if targets['B'] == 5 and targets['C'] == 5:
            trips = ( 'AAAABB', 'BBBCCC', 'CCDDDD' )
            path[1] = get_path( trips )
            targets = {}

    elif n_pl == 17:
        if targets['A'] == 5:
            head_trips += addon( 'A', True )
        elif targets['D'] == 5:
            addon('D')
        elif targets['C'] == 5:
            head_trips += addon( 'C', True )
        else:
            trips = ( 'AAAACC', 'BBBBB', 'CCDDDD' )
            path[1] = get_path( trips )
            targets = {}

    elif n_pl == 16:
        if targets['A'] >= 4:
            if targets['A'] == 5 or min( targets.itervalues() ) != 2:
                head_trips += addon( 'A', True )
        if targets['A']:
            if targets['D'] == 5:
                addon('D')
            elif targets['B'] == targets['C']:
                for k in ( 'B', 'C' ): head_trips += addon( k, True )
            else:
                addon('D')

    elif n_pl == 15:
        if targets['A'] >= 4:
            head_trips += addon( 'A', True )
        elif targets['B'] == 5:
            head_trips += addon( 'B', True )
        elif targets['D'] == 5:
            addon('D')
        elif targets['C'] == 5:
            head_trips += addon( 'C', True )
        else:
            trips = ( 'AAA', 'BBCCCC', 'BBDDDD' )
            path[1] = get_path( trips )
            targets = {}

    if n_pl == 14 or n_pl == 13:
        if targets['C'] + targets['D'] <= 6:
            addon( { 'C': targets['C'], 'D': targets['D'] } )
        elif targets['D'] >= 4:
            addon('D')
        elif targets['A'] >= 4:
            head_trips += addon( 'A', True )
        elif targets['C'] == 5:
            head_trips += addon( 'C', True )
        else:
            if targets['A'] == 3:
                trips = ( 'AAA', 'BBCCCC', 'BBDDD' )
            else:
                trips = ( 'AACCCC', 'BBBBB', 'DDD' )
            path[1] = get_path( trips )
            targets = {}

    payloads = ''.join( k*v for k,v in targets.iteritems() )

    if payloads:
        candidates = get_candidate_paths( payloads, path )
        print candidates[:2]
        trips[head_trips:head_trips] = candidates[0][2]
        path[1] = candidates[0][1]

    return trips, ''.join( path )

def get_candidate_paths( payloads, path, noisy=False ):

    leftovers = len( payloads )

    max_trips = None

    if leftovers <= 12:
        max_trips = 2
    else:
        max_trips = 3

    map = Map(1)
    candidates = []
    min_cost = 80
    phead = path[0]
    ptail = path[2]

    for partition in get_partitions( leftovers, 4, 6, max_trips ):
        seen_p = {}
        lngrp = len( partition )
        if noisy: print partition,
        for grouping in get_combinations( payloads, partition[::-1] ):
            if lngrp>2 and 5*sum( len(set(g)) for g in grouping )/lngrp > 10: continue
            grouping = sorted( grouping, key=lambda x: x[::-1] )
            p = get_path( grouping )
            try:
                seen_p[p] += 1
                continue
            except KeyError:
                pass
            seen_p[p] = 1
            cost = map.walkcost( phead + p + ptail )
            if cost > min_cost: continue
            min_cost = cost
            candidates.append( ( cost, p, grouping ) )
        if noisy: print "total: %s/%s \t\t\t" % ( len(seen_p), sum(seen_p.itervalues()) )

    candidates.sort( key=lambda x: x[0] )

    return candidates

def test_targets():

    tg_tests = [
        { 'A': 6, 'B': 6, 'C': 6 },
        { 'A': 12, 'B': 8, 'C': 2 },
        { 'A': 6, 'B': 2, 'C': 4 },
        { 'A': 2, 'B': 6, 'C': 4 },
        { 'A': 14, 'B': 3, 'C': 2 },
        { 'A': 3, 'B': 3, 'C': 2 },
        { 'A': 4, 'B': 2, 'C': 1, 'D': 1 },
        { 'A': 8, 'B': 4, 'D': 2 },
        { 'A': 10, 'B': 3, 'C': 13, 'D': 1 },
        { 'A': 10, 'B': 3, 'C': 15 },
        { 'A': 5, 'B': 5, 'C': 1, 'D': 5 },
        { 'A': 5, 'B': 4, 'C': 2, 'D': 5 },
        { 'A': 5, 'B': 4, 'C': 4, 'D': 4 },
        { 'A': 5, 'B': 4, 'C': 4, 'D': 5 },
    ]

    for tg in tg_tests:
        trips, path = decide_path( tg )
        print trips
        print ' -> '.join( i for i in path )
        print

def brute_force(n):
    targets = ( 'A', 'B', 'C', 'D' )

    for task in get_partitions( n, 1, 5, 4 ):
        task = sorted( task )
        tlen = len( task )
        for cns in combinations( targets, tlen ):
            seen_s = set()
            for settei in permutations( task ):
                if settei in seen_s: continue
                seen_s.add( settei )
                tg = { i: settei[cns.index(i)] for i in cns }
                print tg
                print decide_path( tg )
                print

if __name__ == "__main__":

    # test_targets()
    print Map(1).walkpath('HXAXBXCXDH')
    # brute_force(15)