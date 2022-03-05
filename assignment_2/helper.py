import numpy as np
import math

def distp2p(p1,p2):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def line(p1,p2):
    if (p1[0]-p2[0])==0:
        m = 100000
        c = 100000
    else:
        m = (p1[1]-p2[1])/(p1[0]-p2[0])
        c = (p1[0]*p2[1]-p2[0]*p1[1])/(p1[0]-p2[0])
    return m,c

def distp2l(q,p1,p2):
    m,c = line(p1,p2)
    if m ==100000:
        x2 = p1[0]
        y2 = q[1]
    else:
        x2 = (m*q[1]-m*c+q[0])/(1+(m**2))
        y2 = ((m**2)*q[1]+m*q[0]+c)/(1+(m**2))
    q2 = np.array([x2,y2])
    return distp2p(q,q2)

def distp2s(q,p1,p2):
    m,c = line(p1,p2)
    # print(q[1])
    if m ==100000:
        x2 = p1[0]
        y2 = q[1]
    else:
        x2 = (m*q[1]-m*c+q[0])/(1+(m**2))
        y2 = ((m**2)*q[1]+m*q[0]+c)/(1+(m**2))
    # print(x2,y2,'x2y2')
    q2 = np.array([x2,y2])
    d1 = distp2p(q2,p1)
    d2 = distp2p(q2,p2)
    d3 = distp2p(p1,p2)
    if d1<d3 and d2<d3:
        return distp2p(q,q2),0
    else:
        if d1>d2:
            return distp2p(p2,q),1
        if d2>d1:
            return distp2p(p1,q),2

def distp2pol(pol,q):
    n = len(pol)
    d = []
    dis = []
    w = []
    for i in range(n):
        i1 = (i+1)%n
        di,wi = distp2s(q,pol[i],pol[i1])
        disi = distp2p(q,pol[i])
        dis.append(disi)
        d.append(di)
        w.append(wi)

    d = np.array(d)
    dis = np.array(dis)
    w = np.array(w)
    # print(d,"d")
    index = np.argmin(d)
    # print(np.min(d),w[index],index)
    return np.min(d),w[index],index,dis

def tanvec(pol,q):
    n = len(pol)
    _,w,ind,dis = distp2pol(pol,q)
    # print(ind)
    if w==0:
        ind1 = (ind+1)%n
        b,p=(pol[ind1]-pol[ind])/np.sqrt(np.sum(np.square(pol[ind1]-pol[ind])))
        return b,p
    else:
        index = np.argmin(dis)
        bo,po = (q-pol[index])/np.sqrt(np.sum(np.square(q-pol[index])))
        b = -po
        p = bo
        return b,p

def perpvec(pol,q):
    _,w,ind,dis = distp2pol(pol,q)
    index = np.argmin(dis)
    b,p = (pol[index]-q)/np.sqrt(np.sum(np.square(pol[index]-q)))
    return b,p

def pot_distp2s(q,p1,p2):
    m,c = line(p1,p2)
    # print(q[1])
    if m ==100000:
        x2 = p1[0]
        y2 = q[1]
    else:
        x2 = (m*q[1]-m*c+q[0])/(1+(m**2))
        y2 = ((m**2)*q[1]+m*q[0]+c)/(1+(m**2))
    # print(x2,y2,'x2y2')
    q2 = np.array([x2,y2])
    d1 = distp2p(q2,p1)
    d2 = distp2p(q2,p2)
    d3 = distp2p(p1,p2)
    if d1<d3 and d2<d3:
        return distp2p(q,q2),q2
    else:
        if d1>d2:
            return distp2p(p2,q),p2
        if d2>d1:
            return distp2p(p1,q),p1

def pot_distp2pol(pol,q):
    n = len(pol)
    cp = []
    d = []
    for i in range(n):
        i1 = (i+1)%n
        di,cpi = pot_distp2s(q,pol[i],pol[i1])
        d.append(di)
        cp.append(cpi)
    d = np.array(d)
    index = np.argmin(d)
    return np.min(d),cp[index]
