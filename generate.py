import itertools

R = 255
G = 0
B = 0

deltas1 = [2,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3]
deltas2 = [3,2,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,4]
deltasR1 = [5,5,5,6,5,5,6,5,5,6,5,5,6,5,5,6,5,5,5,6,5,5,6,5,5,6,5,5,6,5,5,7]
deltasGB = [2,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,3,2,3,2,5,105,5,6,5,5,6,5,5,6,5,5,6,5,5,6,5,5,5,6,5,5,6,5,5,6,5,6,5,5,6]
i1 = itertools.cycle(deltas1)
i2 = itertools.cycle(deltas2)
iR1 = itertools.cycle(deltasR1)
iGB = itertools.cycle(deltasGB)

with open('generated_by_python.txt', 'w') as f:
    f.write('unsigned char precalculated_values[][3] = {\n')
    f.write("{%d,%d,%d},\n" % (R,G,B))
    while R > 173:
        d = next(i1)
        R -= d
        G += d
        f.write("{%d,%d,%d},\n" % (R,G,B))
    while G < 171:
        d = next(i2)
        R = 171
        G += d
        f.write("{%d,%d,%d},\n" % (R,G,B))
    while R > 0:
        dR = next(iR1)
        R -= dR
        G += next(i1)
        f.write("{%d,%d,%d},\n" % (R,G,B))
    while G > 0:
        dGB = next(iGB)
        G -= dGB
        B += dGB
        f.write("{%d,%d,%d},\n" % (R,G,B))


    f.write('};\n')
    
    

    