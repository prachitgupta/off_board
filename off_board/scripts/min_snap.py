# Minimum snap trajectory generation Python implementation 
#require to install Scipy,qpsolvers (python3) 
from math import atan
from math import factorial as f
import numpy as np
from Astar import astar
from scipy.linalg import block_diag
from qpsolvers import solve_qp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import warnings
import pickle
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

class min_snap:          
    def __init__(self, x, y, z, v, n=8):
        self.v = v
        self.n = n         #number of coefficients in each polynomial ( [degree=7] +1 )
        self.m = len(x)-1  # number of peicewise polynomials
        m = self.m
        
    #waypoint arrays
        self.x = x  
        self.y = y
        self.z = z
        self.t=self.time_array(0.1)

  

        #print(self.t)
        self.q=np.zeros(shape=(n*m,1)).reshape((n*m,))
        self.G=np.zeros(shape=((4*m)+2,n*m))
        self.h=np.zeros(shape=((4*m)+2,1)).reshape(((4*m)+2,))
        

        b_x = np.array([x[0],0,0,x[m],0,0])
        b_x = np.append(b_x, x[1:m])
        b_x = np.append(b_x, np.zeros(shape=(3*(m-1))))

        b_y = np.array([y[0],0,0,y[m],0,0])
        b_y = np.append(b_y, y[1:m])
        b_y = np.append(b_y, np.zeros(shape=(3*(m-1))))

        b_z = np.array([z[0],0,0,z[m],0,0])
        b_z = np.append(b_z, z[1:m])
        b_z = np.append(b_z, np.zeros(shape=(3*(m-1))))

        self.b_x = b_x
        self.b_y = b_y
        self.b_z = b_z

        self.form_Q()
        self.form_A()

    def time_array(self,start_time):      # generate timestamp array based on given velocity profile
        t = [start_time]
        for i in range(1,self.m+1):
            dist = np.sqrt((self.x[i]-self.x[i-1])**2 + (self.y[i]-self.y[i-1])**2 + (self.z[i]-self.z[i-1])**2)
            ti = dist/self.v
            t.append(t[-1]+ti)
        return t

    def form_Q(self):  #generate the Q matirx
        Q_list = []
        for l in range(1,self.m+1):
            
            Q_i=np.zeros(shape=(self.n,self.n))
            for i in range(self.n):       
                for j in range(self.n):
                    if (((i>3) and (j<=3))) or (((j>3) and (i<=3))):
                        Q_i[i][j]=0
                    elif (i<=3) and (j<=3):
                        Q_i[i][j]=0
                    else:
                        r,c=i+1,j+1
                        Q_i[i][j]= (f(r)*f(c)*(pow(self.t[l],r+c-7)-pow(self.t[l-1],r+c-7)))/(f(r-4)*f(c-4)*(r+c-7))
            Q_list.append(Q_i)
        Q = Q_list[0]
        Q_list.pop(0)
        for i in Q_list:
            Q=block_diag(Q,i)
        self.Q=Q+(0.0001*np.identity(self.n*self.m))
        print(type(self.Q),'typeofQ')

    def form_A(self):   #form the A matrix
        n = self.n
        m = self.m
        t = self.t
        A=np.zeros(shape=((4*m)+2,n*m))

        for j in range(n*m):
                if j>=n: #first n terms non zero
                    A[0][j],A[1][j],A[2][j]=0,0,0
                else:
                    A[0][j],A[1][j],A[2][j]=pow(t[0],j),j*pow(t[0],j-1),j*(j-1)*pow(t[0],j-2)

        for j in range(n*m):
                if j<n*(m-1): #last n terms non zero
                    A[3,j],A[4,j],A[5,j]=0,0,0
                else:
                    h=n*(m-1)
                    A[3,j],A[4,j],A[5,j]=pow(t[m],j-h),(j-h)*pow(t[m],j-h-1),(j-h)*(j-h-1)*pow(t[m],j-h-2)
        z=[]
        for i in range(1,m):
            h=[]
            for j in range(n*m):
                if (j<((i-1)*n)) or (j>=(i*n)):
                    h.append(0)
                else:
                    h.append(pow(t[i],j-((i-1)*n)))
            z.append(h)    
        A[6:(6+m-1)]=z
        pva_const=[]
        for i in range(1,m):
            x_i,v_i,a_i=[],[],[]
            for j in range(n*m):
                if (j<((i-1)*n)) or (j>=((i+1)*n)):
                    x_i.append(0)
                    v_i.append(0)
                    a_i.append(0)
                    
                elif (j<((i)*n)) and (j>=((i-1)*n)):
                    x_i.append(pow(t[i],j-((i-1)*n)))
                    v_i.append((j-((i-1)*n))*pow(t[i],j-1-((i-1)*n)))
                    a_i.append((j-1-((i-1)*n))*(j-((i-1)*n))*pow(t[i],j-2-((i-1)*n)))
                else:
                    x_i.append((-1)*pow(t[i],j-((i)*n)))
                    v_i.append((-1)*(j-((i)*n))*pow(t[i],j-1-((i)*n)))
                    a_i.append((-1)*(j-1-((i)*n))*(j-((i)*n))*pow(t[i],j-2-((i)*n)))
            pva_i=[x_i,v_i,a_i]        
            pva_const=pva_const+pva_i
        A[(6+m-1):]=pva_const
        self.A = A
        print(type(self.A),'typeofA')

    def solve(self):     # Solve for coefficient array using pthon QP Solvers
        print(type(self.q),type(self.G),type(self.h),type(self.b_x))
        self.p_x=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_x, solver="osqp")
        self.p_y=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_y, solver="osqp")
        self.p_z=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_z, solver="osqp")

    def plot(self):      #plot trajectory using result arrays self.p_x,self.p_y,self.p_z
        plt.figure(figsize=(10,5))
        ax = plt.axes(projection ='3d')

        ax.scatter(self.x, self.y, self.z, 'b',marker='o')
        for v in range(self.m):
            x_des,vx,Ax,y_des,vy,Ay,z_des,vz,Az,dt,yaw_d=[],[],[],[],[],[],[],[],[],[],[]
            
            r=np.linspace(self.t[v],self.t[v+1],100) #break time between any two waypoints in 100 segments
            time=(self.t[v+1]-self.t[v])/100
            for i in range(100):
                g,g_dot,g_ddot,e,e_dot,e_ddot,f,f_dot,f_ddot=0,0,0,0,0,0,0,0,0 #basically trajectory in x,y,z
                for j in range(self.n*v,(v+1)*self.n):
                    g1=g
                    e1=e
                    #basically g  is list of polynomial eq of all 100 pieces of a given segment, in other worgs trajectory
                    g=g+(self.p_x[j]*pow(r[i],j-(self.n*v))) 
                    e=e+(self.p_y[j]*pow(r[i],j-(self.n*v)))
                    f=f+(self.p_z[j]*pow(r[i],j-(self.n*v)))

                    #basically  g_dot is list of polynomial eq of all 100 pieces of a given segment, in other worgs trajectory
                    g_dot=g_dot+(self.p_x[j]*(j-self.n*v)*pow(r[i],j-1-(self.n*v))) 
                    e_dot=e_dot+(self.p_y[j]*(j-self.n*v)*pow(r[i],j-1-(self.n*v)))
                    f_dot=f_dot+(self.p_z[j]*(j-self.n*v)*pow(r[i],j-1-(self.n*v)))

                    #basically g_ddot  is list of polynomial eq of all 100 pieces of a given segment, in other worgs trajectory
                    g_ddot=g_ddot+(self.p_x[j]*(j-self.n*v)*(j-1-self.n)*pow(r[i],j-2-(self.n*v))) 
                    e_ddot=e_ddot+(self.p_y[j]*(j-self.n*v)*(j-1-self.n)*pow(r[i],j-2-(self.n*v)))
                    f_ddot=f_ddot+(self.p_z[j]*(j-self.n*v)*(j-1-self.n)*pow(r[i],j-2-(self.n*v)))

                    dg=(g-g1)/time 
                    de=(e-e1)/time 
                    yaw=atan(de/dg)

                yaw_d.append(yaw)
                dt.append(time)    
                x_des.append(g) #x_des
                y_des.append(e)#y_des
                z_des.append(f)#z_des

                vx.append(g_dot) #vx_des
                vy.append(e_dot)
                vz.append(f_dot)

                Ax.append(g_ddot) #Ax_des
                Ay.append(e_ddot)
                Az.append(f_ddot)

            ax.plot3D(x_des,y_des,z_des, 'r')
        traj=np.array((x_des,0,0,y_des,0,0,z_des,0,0,yaw_d)) 
        plt.show()
        return traj

if __name__ == '__main__':
   # Get Waypoints
    maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (9, 5)
    waypoints = astar(maze, start, end)
    i = 0
    x ,y,z = [],[],[]
    while i < len(waypoints):
        x.append(waypoints[i][0])
        y.append(waypoints[i][1])
        z.append(5)
        i += 1
    
    v = 10                     # average velocity profile
    ms = min_snap(x,y,z,v)     
    ms.solve()                 #solve for polynomial coefficients
    gen=ms.plot()
    with open('pickled_traj.txt','wb') as gt:
        pickle.dump(gen, gt)

