import matplotlib.pyplot as plt
import numpy as np
from numpy import sin as S 
from numpy import cos as C 
from numpy import tan as T
from numpy.core.shape_base import block 
from scipy.linalg import block_diag
from scipy.integrate import odeint
import math

class controller:
    def __init__(self):
        self.x, self.y, self.z, self.xdot, self.ydot, self.zdot, self.theta, self.phi, self.sci, self.thetadot, self.scidot, self.phidot = np.array([0]),np.array([5]),np.array([0]),np.array([0]),np.array([0]),np.array([0]),np.array([0]),np.array([0]),np.array([0]),np.array([0]),np.array([0]),np.array([0]) 
        self.x5d = np.array([0]) 
        self.x5dot = np.array([0])
        self.g = 9.81
        self.m = 2.0
        self.Jx, self.Jy = 0.018125, 0.018125
        self.Jz = 0.035
        self.Jr = 6*0.001
        self.Jb = np.diag([self.Jx,self.Jy,self.Jz])
        self.omegabar = 1     ###to be tuned
        self.k1, self.k2, self.k3 = 3, 4, 10   ###to be tuned
        self.c1, self.c2, self.c3, self.c4, self.c5, self.c6, self.c7, self.c8, self.c9, self.c10 = 5, 10, 12, 7, 6, 8, 9, 11, 15, 8
        self.x1ddotdot, self.x3ddotdot, self.x5ddotdot = np.array([0]), np.array([0]), np.array([0])
        self.dphi, self.dtheta, self.dsci = 1.2, 1.2, 1.2
        self.dx, self.dy, self.dz = .3, .1, 0.9
        self.rho1, self.rho2, self.rho3, self.rho4, self.rho5, self.rho6 = 1,1,0.1,0.1,0.0001,0.4 #to be tuned 
        self.dt = 0.1
        self.t = 0.0
        self.U1, self.U2, self.U3, self.U4 = np.array([0]),np.array([0]),np.array([0]),np.array([0])
        ####
        self.x3dot = 0
        self.x5dot = 0
        self.x5ddot = 0
        self.v1o,self.v2o,self.v3o,self.v4o,self.v5o,self.v6o = np.array([0.0]),np.array([0.0]),np.array([0.0]),np.array([0.0]),np.array([0.0]),np.array([0.0])
        self.x5d = np.array([0])
        self.x5ddot = np.array([0])
        self.x7ddot, self.x7d = np.array([0]), np.array([0])
        self.x9ddot, self.x9d = np.array([0]), np.array([0])
        self.x11d = np.array([0])
        self.x11ddot = np.array([0])
        self.x1ddot = np.array([0])
        self.x3ddot = np.array([0])
        self.v1,self.v2,self.v3,self.v4,self.v5,self.v6 = np.array([0.0]),np.array([0.0]),np.array([0.0]),np.array([0.0]),np.array([0.0]),np.array([0.0])
        self.a1,self.a2,self.a3,self.a4,self.a5,self.a6,self.a7,self.a8,self.a9,self.a10,self.a11 = 0,0,0,0,0,0,0,0,0,0,0


    def calc_input(self):
        
        #value of constant ai
        self.a1 = (self.Jy-self.Jz)/self.Jx
        self.a2 = (self.Jr)/self.Jx
        self.a3 = self.dphi/self.Jx
        self.a4 = (self.Jz-self.Jx)/self.Jy
        self.a5 = self.Jr/self.Jy
        self.a6 = self.dphi/self.Jy
        self.a7 = (self.Jx-self.Jy)/self.Jz
        self.a8 = self.dsci/self.Jz
        self.a9 = self.dx/self.m
        self.a10 = self.dy/self.m
        self.a11 = self.dz/self.m 

        #calc virtual control input
        self.v1 = self.x7ddot + self.rho1*(self.x7d - self.x7)
        self.v1dot = self.derivative(self.v1,self.v1o)
        self.v2 = self.v1dot + (self.x7d - self.x7) + self.a9*self.x8 + self.rho2*(self.v1-self.x8)
        self.v3 = self.x9ddot + self.rho3*(self.x9d-self.x9)
        self.v3dot = self.derivative(self.v3,self.v3o) 
        self.v4 = self.v3dot + (self.x9d - self.x9) + self.a10*self.x10 + self.rho4*(self.v3 - self.x10)
        self.v5 = self.x11ddot + self.rho5*(self.x11d - self.x11) 
        self.v5dot = self.derivative(self.v5,self.v5o) 
        self.v6 = self.v5dot + (self.x11d - self.x11) + self.g + self.a11*self.x12 + self.rho6*(self.v5 - self.x12)

        #desired states
        #np.seterr(invalid='ignore')
        a = C(self.x5d)
        b = S(self.x5d)
        c = (self.v2 + self.v4)/self.v6
        d = a + b
        self.x1d = np.arctan2((a*c*self.v6-d*self.v4),(a*self.v6*np.sqrt(c**2 + d**2)))
        self.x3d = np.arctan2(c,d)
        self.z1 = self.x1d - self.x1
        self.z1dot = self.x1ddot - self.x1dot
        self.z3 = self.x3d - self.x3
        self.z3dot = self.x3ddot - self.x3dot
        self.z5 = self.x5d - self.x5
        self.z5dot = self.x5ddot - self.x5dot
        self.U4 = (a**2*(c**2+d**2)*self.v6**2 + (a*c*self.v6 - d*self.v4)**2)/(a*d)
        self.U1 = self.x1ddotdot + self.c1*(self.z1dot) - (self.a1*self.x4*self.x6 + self.a2*self.omegabar*self.x4 - self.a3*self.x2) + self.k1*np.sign(self.z1dot + self.c1*self.z1)
        self.U2 = self.x3ddotdot + self.c2*(self.z3dot) - (self.a4*self.x2*self.x6 + self.a5*self.omegabar*self.x2 - self.a6*self.x4) + self.k2*np.sign(self.z3dot + self.c4*self.z3)
        self.U3 = self.x5ddotdot + self.c3*(self.z5dot) - (self.a7*self.x2*self.x4  - self.a8*self.x6) + self.k3*np.sign(self.z5dot + self.c10*self.z5)
        # print(f"\n{a} + { b} + {c} + {d}")

    def integrate(self,q1,q2):
        return q2*self.dt + q1

    def derivative(self,q1,q2):
        return (q1-q2)/self.dt

    def update(self):
        #introduction of all states and control inputs      
        self.x1 = self.phi
        self.x2 = self.phidot
        self.x3 = self.theta
        self.x4 = self.thetadot
        self.x5 = self.sci
        self.x6 = self.scidot
        self.x7 = self.x
        self.x8 = self.xdot
        self.x9 = self.y
        self.x10 = self.ydot
        self.x11 = self.z
        self.x12 = self.zdot

        #differential parameters
        self.x1dot = self.x2
        self.x2dot = self.a1*self.x4*self.x6 + self.a2*self.omegabar*self.x4 - self.a3*self.x2 + self.U1
        self.x3dot = self.x4
        self.x4dot = self.a4*self.x2*self.x6 + self.a5*self.omegabar*self.x2 - self.a6*self.x4 + self.U2
        self.x5dot = self.x6
        self.x6dot = self.a7*self.x2*self.x4 - self.a8*self.x6 + self.U3
        self.x7dot = self.x8
        self.x8dot = (C(self.x1)*S(self.x3)*C(self.x5) + S(self.x1)*S(self.x5))*self.U4 - self.a9*self.x8
        self.x9dot = self.x10
        self.x10dot = (C(self.x1)*S(self.x3)*S(self.x5) - S(self.x1)*S(self.x5))*self.U4 - self.a10*self.x10
        self.x11dot = self.x12
        self.x12dot = -self.g + (C(self.x1)*C(self.x3))*self.U4 - self.a11*self.x12

    def system_dynamics(self, state, t):
        self.phi, self.phidot, self.theta, self.thetadot, self.sci, self.scidot, self.x, self.xdot, self.y, self.ydot, self.z, self.zdot = state
        self.update()
        self.calc_input()
        return [self.x1dot, self.x2dot, self.x3dot, self.x4dot, self.x5dot, self.x6dot, self.x7dot, self.x8dot, self.x9dot, self.x10dot, self.x11dot, self.x12dot]
    
    def compute(self):
        T = np.linspace(0, 200, 2000)
        self.update()

        plt.figure(figsize=(10, 5))
        ax = plt.axes(projection='3d')
        errors = []

        for i in range(1, 200):
            self.t = T[i]
            self.v1o, self.v2o, self.v3o, self.v4o, self.v5o, self.v6o = np.copy(self.v1), np.copy(self.v2), np.copy(self.v3), np.copy(self.v4), np.copy(self.v5), np.copy(self.v6)
            self.x5d = np.array([0])
            self.x5ddot = np.array([0])
            self.x7ddot, self.x7d = np.array([5*C(self.t)]), np.array([5*S(self.t)])
            self.x9ddot, self.x9d = np.array([-5*S(self.t)]), np.array([5*C(self.t)])
            self.x11d = np.array([0.2*self.t])
            self.x11ddot = np.array([0.2])
            self.calc_input()

            init_state = np.array([self.phi, self.phidot, self.theta, self.thetadot, self.sci, self.scidot, self.x, self.xdot, self.y, self.ydot, self.z, self.zdot]).reshape(-1)
            sol = odeint(self.system_dynamics, init_state, [T[i - 1], T[i]])
            self.phi, self.phidot, self.theta, self.thetadot, self.sci, self.scidot, self.x, self.xdot, self.y, self.ydot, self.z, self.zdot = sol[-1]

            self.update()

            ax.plot(self.x, self.y, self.z, c='lightblue', marker='o')
            ax.plot(5*S(self.t), 5*C(self.t), 0.2*self.t, c='red', marker='o')
            # errors.append(math.sqrt(((self.x - 5*S(self.t))/(5*S(self.t)))**2 + ((self.y - 5*C(self.t))/(5*C(self.t)))**2 + ((self.z - 0.2*self.t)/(0.2*(self.t)))))
            plt.show(block=False)
            plt.pause(0.1)
        # plt.show()
        # plt.plot(errors)
        plt.show()

if __name__ == '__main__':
    nlc = controller()
    nlc.compute()
