import numpy as np
import matplotlib.pyplot as plt
from math import *

x_c = [0,5,5,0,0]
y_c = [0,0,5,5,0]
accept_rad = 0.5

def two_points(x1,y1,x2,y2,d):
    if x1!=x2 and y1!=y2:
        a = 1/(x1-x2)
        b = 1/(y1-y2)
        A = b
        B = -a
        C = -b*x2 + a*y2
        t = d/np.sqrt(A**2 + B**2)
        xa = x2 + B*t
        xb = x2 - B*t
        ya = y2 - A*t
        yb = y2 + A*t
        return xa,ya,xb,yb 
    if x1==x2:
        return x2+d,y2,x2-d,y2
    if y1==y2:
        return x2,y2+d,x2,y2-d
def line_sign(x1,y1,x2,y2,x0,y0):
    if x1!=x2 and y1!=y2:
        return np.sign( (x0-x1)/(x1-x2) + (y0-y1)/(y1-y2) )
    if x1==x2:
        return np.sign(x0-x1)
    if y1==y2:
        return np.sign(y0-y1)


def pseudo_wps(x, y):
    pswpx = []
    pswpy = []
    for i in range(len(x)):
        nton = (i+2)%(len(x)-1)
        n = (i+1)%(len(x)-1)
        sgn = line_sign(x[i],y[i],x[n],y[n],x[nton],y[nton])
        xa,ya,xb,yb = two_points(x[i],y[i],x[n],y[n],accept_rad)
        sgna = line_sign(x[i],y[i],x[n],y[n],xa,ya)
        sgnb = line_sign(x[i],y[i],x[n],y[n],xb,yb)
        if sgnb==sgn:
            pswpx.append(xa)
            pswpy.append(ya)
        else:
            pswpx.append(xb)
            pswpy.append(yb)
    return pswpx,pswpy

x_n,y_n = pseudo_wps(x_c,y_c)
x_n.insert(0,x_n[len(x_n)-1])
x_n.pop(len(x_n)-1)
y_n.insert(0,y_n[len(y_n)-1])
y_n.pop(len(y_n)-1)
    
plt.plot(x_c,y_c,label='actual')
plt.plot(x_n,y_n,label='new')
plt.legend()
plt.show()


