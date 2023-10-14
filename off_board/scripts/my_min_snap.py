#programming guide = https://www.programmersought.com/article/79683831846/
#hand wavy explanation for beginners(notes) = https://drive.google.com/file/d/1yBmen2WqSF6vAuW6x42-PIBodAUoeO_R/view?usp=sharing

from math import atan
from math import factorial as f
import numpy as np

from scipy.linalg import block_diag
from qpsolvers import solve_qp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import warnings
import pickle
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

class min_snap :
    def __init__(self,x,y,z,v):
        self.x = x
        self.y = x
        self.z = x
        self.v = v

        self.n = 8 #minimizing snap generates a 7 deg piecewise poly 
        self.m = len(x) - 1 #no. of segments = no. of waypoints -1
        self.t = self.cumTime(0.1)
        m = self.m
        n = self.n

        #form A {matrix containing coefficient of equality constrains}

        self.form_A()

        #form B {column matrix containing results of equality constrains}, beware of filling in same order as A

        b_x = np.array([x[0],0,0,x[m],0,0]) #boundary conditions
        b_x = np.append(b_x, x[1:m]) #m-1 intermediate equality constrain
        b_x = np.append(b_x, np.zeros(shape=(3*(m-1)))) #3(m-1) constrain on smooth trajectory

        b_y = np.array([y[0],0,0,y[m],0,0])
        b_y = np.append(b_y, y[1:m])
        b_y = np.append(b_y, np.zeros(shape=(3*(m-1))))

        b_z = np.array([z[0],0,0,z[m],0,0])
        b_z = np.append(b_z, z[1:m])
        b_z = np.append(b_z, np.zeros(shape=(3*(m-1))))

        self.b_x = b_x
        self.b_y = b_y
        self.b_z = b_z

        #Equility constrain sorted AP = B

        # generate Q {matrix to optimize} refer to programming guide to know how it came into picture
        self.form_Q()

        #inequality constrain , set to 0 in this code
        self.q=np.zeros(shape=(n*m,1)).reshape((n*m,))
        self.G=np.zeros(shape=((4*m)+2,n*m))
        self.h=np.zeros(shape=((4*m)+2,1)).reshape(((4*m)+2,))


    def form_A(self):
        m = self.m
        n = self.n
        t = self.t

        A = np.zeros(4*m+2 ,n*m)  #4m + 2 constrains and n*m coefficients

        #first 3 constraints on starting point
        for j in range(n*m):
            if j>=n: #first n terms non zero
                A[0,j],A[1,j],A[2,j] = 0,0,0
            else:
                A[0,j],A[1,j],A[2,j] = pow(t[0],j), j*pow[t[0],j-1], (j)*(j-1)*pow(t[0],j-2)

        #3 constraints on ending point {va =0}
        for j in range(n*m):
                if j<n*(m-1):  #last n terms non zero
                    A[3,j],A[4,j],A[5,j]=0,0,0
                else:
                    h=n*(m-1)
                    A[3,j],A[4,j],A[5,j]=pow(t[m],j-h),(j-h)*pow(t[m],j-h-1),(j-h)*(j-h-1)*pow(t[m],j-h-2)

        #m-1 equality constrains on intermediate points {p = waypoint_i}
        z=[]
        for i in range(1,m): #keeps track of rows
            h=[] 
            for j in range(n*m): #keeps track of columns
                if (j<((i-1)*n)) or (j>=(i*n)): #non zero n terms , position varies acc to row no.
                    h.append(0)
                else:
                    h.append(pow(t[i],j-((i-1)*n)))
            z.append(h)    
        A[6:(6+m-1)]=z

        #3(m-1), continuity,differentiability and double differentiability (hence 3) constrains on m-1 middle points
        pva_const=[]
        for i in range(1,m):
            x_i,v_i,a_i=[],[],[]
            for j in range(n*m):
                if (j<((i-1)*n)) or (j>=((i+1)*n)):#2n non zero terms { Pi+1 -Pi} ,
                    x_i.append(0)
                    v_i.append(0)
                    a_i.append(0)
                    
                elif (j<((i)*n)) and (j>=((i-1)*n)):#n non zero terms corresponding to first poly piece {Pi}
                    x_i.append(pow(t[i],j-((i-1)*n)))
                    v_i.append((j-((i-1)*n))*pow(t[i],j-1-((i-1)*n)))
                    a_i.append((j-1-((i-1)*n))*(j-((i-1)*n))*pow(t[i],j-2-((i-1)*n)))
                else: #n non zero terms corresponding to second poly piece{Pi+!}
                    x_i.append((-1)*pow(t[i],j-((i)*n)))
                    v_i.append((-1)*(j-((i)*n))*pow(t[i],j-1-((i)*n)))
                    a_i.append((-1)*(j-1-((i)*n))*(j-((i)*n))*pow(t[i],j-2-((i)*n)))
            pva_i=[x_i,v_i,a_i]        
            pva_const=pva_const+pva_i
        A[(6+m-1):]=pva_const

        #A is ready
        self.A = A


    def form_Q(self):  #generate the Q matirx
        Q_list = []
        for l in range(1,self.m+1):
            
            Q_i=np.zeros(shape=(self.n,self.n))
            for i in range(self.n):#rows
                for j in range(self.n):#columns
                    if (((i>3) and (j<=3))) or (((j>3) and (i<=3))): #4*4 submatrix  and 4*(n-3) submatrix = 0
                        Q_i[i,j]=0
                    elif (i<=3) and (j<=3): #(n-3)*4 submatrix = 0
                        Q_i[i][j]=0
                    else:
                        r,c=i+1,j+1 #r and c are row and column index
                        Q_i[i][j]= (f(r)*f(c)*(pow(self.t[l],r+c-7)-pow(self.t[l-1],r+c-7)))/(f(r-4)*f(c-4)*(r+c-7))
            Q_list.append(Q_i)
        Q = Q_list[0]
        Q_list.pop(0)
        for i in Q_list:
            Q=block_diag(Q,i) #converts list into block diag = https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.block_diag.html
        self.Q=Q+(0.0001*np.identity(self.n*self.m))
        print(type(self.Q),'typeofQ')

    def solve(self):     # Solve for coefficient array using pthon QP Solvers 
        #to know format of standard QP =  https://pypi.org/project/qpsolvers
        self.p_x=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_x, solver="osqp")
        self.p_y=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_y, solver="osqp")
        self.p_z=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_z, solver="osqp")   


    #generated time array
    def cumTime(self,t0): #note: not time required to travel a segment but total time to reach a waypoint
        t = [t0]
        #assuming uniform velocity distribution , time allocated wrt distance
        for i in range(self.m):
            dis_mth = np.sqrt((x[i+1]- x[i])**2 + (y[i+1]- y[i])**2 + (z[i+1]- z[i])**2 )
            t_mth = dis_mth/self.v
            t = t.append(t[-1] + t_mth)
        return t

    #plot the result
    def plot(self):  #plot trajectory using result arrays self.p_x,self.p_y,self.p_z
        plt.figure(figsize=(10,5))
        ax = plt.axes(projection ='3d')

        ax.scatter(self.x, self.y, self.z, 'b',marker='o')
        for v in range(self.m):
            w,u,a,dt,yaw_d=[],[],[],[],[]
            
            r=np.linspace(self.t[v],self.t[v+1],100)
            time=(self.t[v+1]-self.t[v])/100
            for i in range(100):
                g,e,f=0,0,0
                for j in range(self.n*v,(v+1)*self.n):
                    g1=g
                    e1=e
                    g=g+(self.p_x[j]*pow(r[i],j-(self.n*v)))
                    e=e+(self.p_y[j]*pow(r[i],j-(self.n*v)))
                    f=f+(self.p_z[j]*pow(r[i],j-(self.n*v)))
                    dg=(g-g1)/time 
                    de=(e-e1)/time 
                    yaw=atan(de/dg)
                yaw_d.append(yaw)
                dt.append(time)    
                w.append(g)
                u.append(e)
                a.append(f)
            ax.plot3D(w, u, a, 'r')
        traj=np.array((dt,w,u,a,yaw_d))   
        plt.show()
        return traj



if __name__ == '__main__':
    x = [0,1,2,3,4,5,6,7,8,9]             # define waypoint array
    y = [0,1,2,3,4,5,6,7,8,9]
    z = [5,5,5,5,5,5,5,5,5,5]
    v = 10                     # average velocity profile
    ms = min_snap(x,y,z,v)     
    ms.solve()                 #solve for polynomial coefficients
    gen=ms.plot()
    with open('pickled_traj.txt','wb') as gt:
        pickle.dump(gen, gt)