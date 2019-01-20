import numpy as np
import matplotlib.pyplot as plt

fig, ax = plt.subplots()

def t(y):
    x = y[y.find('(')+1:y.find(')')]
    a = x[:x.find(',')]
    b = x[x.find(',')+1:]
    return (float(a), float(b))

def s(y, c):
    face_x = [x[0] for x in y]
    face_y = [x[1] for x in y]
    face_x.append(face_x[0])
    face_y.append(face_y[0])
    ax.plot(face_x, face_y, c, markersize=20)
    
def PolygonArea(corners):
    n = len(corners) # of corners
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += corners[i][0] * corners[j][1]
        area -= corners[j][0] * corners[i][1]
    area = area / 2.0
    return area

def all(f, a):
    faces = []
    face_file = open(f)
    p = []
    for x in face_file:
        if x.find('(') > -1:
            p.append(t(x))
        elif len(p) > 0:
            faces.append(p)
            p = []

    if len(p) > 0:
        faces.append(p)

    for i in range(0,len(faces)):
        s(faces[i], a[i])

all("orig.txt",['k','k','r:','r:','b:','b:','b:'])

#ax.axis('equal')
ax.set_aspect('equal', 'box')

plt.show()
