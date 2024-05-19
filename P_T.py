import roboticstoolbox as rtb 
import numpy as np 
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

## Create Model
#link lengths in mm
a1 = float(input("a1 ="))
a2 = float(input("a2 ="))
a3 = float(input("a3 ="))
a4 = float(input("a4 ="))
a5 = float(input("a5 ="))


#link conversion to meters
def mm_to_meter(a):
    m = 1000 #1 meter = 1000
    return a/m 

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2)
a3 = mm_to_meter(a3)
a4 = mm_to_meter(a4)
a5 = mm_to_meter(a5)


# limit of variable d1
lm1 = float(input("lm1 = "))
lm1 = mm_to_meter(lm1)

#Create links
#robot_variable = DHRobot([RevoluteDH(d,r,alpha,offset=theta,qlim)])
#robot_variable = DHRobot([PrismaticDH(d=0,r,alpha,offset=d,qlim)])
SCARA = DHRobot([
   PrismaticDH(0,0,0,0,qlim=[0,0]),
   RevoluteDH(a1,0,(0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    PrismaticDH(0,a2,0,0,qlim=[0,0]),
    RevoluteDH(a3,0,(180/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
    PrismaticDH(0,a4,0,0,qlim=[0,0]),
    PrismaticDH(0,(0/180.0)*np.pi,(0/180.0)*np.pi,a5,qlim=[0,0]),
    
], name = "SCARA")

print(SCARA)

def deg_to_rad(T):
    return (T*np.pi)/180


q0 = np.array([0,0,0,0,0,0])

q1 = ([0,
        deg_to_rad(float(input("t1 = "))),
        0,
       deg_to_rad(float(input("t2 ="))),
       0,
       mm_to_meter(float(input("d3 =")))])

q2 = ([0,
        deg_to_rad(float(input("t1 = "))),
        0,
      deg_to_rad(float(input("t2 ="))),
      0,
       mm_to_meter(float(input("d3 =")))])

q3 = ([0,
        deg_to_rad(float(input("t1 = "))),
        0,
       deg_to_rad(float(input("t2 ="))),
       0,
       mm_to_meter(float(input("d3 =")))])

traj1 = rtb.jtraj(q0,q1,20)
traj2 = rtb.jtraj(q1,q2,20)
traj3 = rtb.jtraj(q2,q3,20)

x1 = -0.1
x2 = 0.1
y1 = -0.1
y2 = 0.1
z1 = 0.0
z2 = 0.1


SCARA.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
SCARA.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
SCARA.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2],block=True)