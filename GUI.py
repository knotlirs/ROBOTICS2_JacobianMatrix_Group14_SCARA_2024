from tkinter import*
from tkinter import messagebox
from tkinter import PhotoImage
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

# Creating a GUI window with a title
gui = Tk()
gui.title("SCARA Design Calculator")
gui.resizable(False,False)
gui.config(bg="pink")

def reset():
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)
    a5_E.delete(0, END)

    t1_E.delete(0, END)
    t2_E.delete(0, END)
    d3_E.delete(0, END)

    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)

def f_k():
    #link lengths in mm

    a1 = float(a1_E.get())/100

    a2 = float(a2_E.get())/100

    a3 = float(a3_E.get())/100

    a4 = float(a4_E.get())/100

    a5 = float(a5_E.get())/100

    #joint variables: is mm if f, is degrees if theta

    T1 = float(t1_E.get()) 

    T2 = float(t2_E.get()) 

    D3 = float(d3_E.get())/100

    #degree to radian

    T1 = (T1/180.0)*np.pi

    T2 = (T2/180.0)*np.pi

    #Parametric Table (theta, alpha, r, d)

    PT = [[(0.0/180.0)*np.pi + T1,(0.0/180.0)*np.pi,a2,a1], 
    [(0.0/180.0)*np.pi + T2,(180.0/180.0)*np.pi,a4,a3],
    [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a5+ D3]]

    #HTM formula and multiplication

    i = 0

    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])], [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
    [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
    [0,0,0,1]]

    i = 1

    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])], [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
    [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
    [0,0,0,1]]

    i = 2

    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])], [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
    [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
    [0,0,0,1]]

    #Multiply the matrices

    H0_1 = np.matrix(H0_1)

    H1_2 = np.matrix(H1_2)

    H2_3 = np.matrix(H2_3)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)

    X0_3 = H0_3[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_3*100,3))

    Y0_3 = H0_3[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_3*100,3))

    Z0_3 = H0_3[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_3*100,3))
    ## Jacobian Matrix

    # Jacobian Window
    J_sw = Toplevel()
    J_sw.title("Velocity Calculator")
    J_sw.resizable(False,False)

    #1. Linear/Translation Vectors
    Z_1 = [[0],[0],[1]] #The [0,0,1] vector

    #Row 1 to 3, column 1
    J1a = [[1,0,0],
        [0,1,0],
        [0,0,1]]
    J1a = np.dot(J1a,Z_1)
    J1a = np.array(J1a)

    J1b_1 = H0_3[0:3,3:] #d0_3
    J1b_1 = np.array(J1b_1)
    J1b_2 = [[0],[0],[0]]
    J1b_2 = np.array(J1b_2)

    J1b = J1b_1 - J1b_2

    J1 = [[J1a[1,0]*J1b[2,0] - J1a[2,0]*J1b[1,0]],
        [J1a[2,0]*J1b[0,0] - J1a[0,0]*J1b[2,0]],
        [J1a[0,0]*J1b[1,0] - J1a[1,0]*J1b[0,0]]]

    J1 = np.array(J1)
    print("J1 = ")
    print(J1)

    #Row 1 to 3, column 1
    J2a = H0_1[0:3,0:3] #R0_1
    J2a = np.dot(J2a,Z_1)
    J2a = np.array(J2a)

    J2b_1 = H0_3[0:3,3:] #d0_3
    J2b_1 = np.array(J2b_1)
    J2b_2 = H0_1[0:3,3:]
    J2b_2 = np.array(J2b_2)

    J2b = J2b_1 - J2b_2

    J2 = [[J2a[1,0]*J2b[2,0] - J2a[2,0]*J2b[1,0]],
        [J2a[2,0]*J2b[0,0] - J2a[0,0]*J2b[2,0]],
        [J2a[0,0]*J2b[1,0] - J2a[1,0]*J2b[0,0]]]

    print("J2 = ")
    print(J2)

    J3 = H0_2[0:3,0:3] 
    J3 = np.dot(J3,Z_1)
    print("J3 = ")
    print(J3)

    J4 = J1a
    J4 = np.array(J4)
    print("J4 = ")
    print(J4)

    J5 = J2a
    J5 = np.array(J5)
    print("J5 = ")
    print(J5)

    J6 = [[0],[0],[0]]
    J6 = np.array(J6)
    print("J6 = ")
    print(J6)

    #3. Concatenated Jacobian Matrix
    JM1 = np.concatenate((J1,J2,J3),1)
    JM2 = np.concatenate((J4,J5,J6),1)

    J = np.concatenate((JM1,JM2),0)
    J = np.matrix(J)


    def update_velo():
        T1p = T1_slider.get()
        T2p = T2_slider.get()
        D3p = D3_slider.get()

        q = np.array([[T1p],[T2p],[D3p]])
        E = np.dot(J,q)

        xp_e = E[0,0]
        x_entry.delete(0,END)
        x_entry.insert(0,str(xp_e))

        yp_e = E[1,0]
        y_entry.delete(0,END)
        y_entry.insert(0,str(yp_e))

        zp_e = E[2,0]
        z_entry.delete(0,END)
        z_entry.insert(0,str(zp_e))

        ωx_e = E[3,0]
        ωx_entry.delete(0,END)
        ωx_entry.insert(0,str(ωx_e))

        ωy_e = E[4,0]
        ωy_entry.delete(0,END)
        ωy_entry.insert(0,str(ωy_e))

        ωz_e = E[5,0]
        ωz_entry.delete(0,END)
        ωz_entry.insert(0,str(ωz_e))
   
    # Jacobian Sliders

    T1_velo = Label(J_sw,text=("θ1* = "),font=(5)) 
    T1_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
    T1_unit = Label(J_sw,text=("rad/s"),font=(5))

    T2_velo = Label(J_sw,text=("θ2* = "),font=(5)) 
    T2_slider = Scale(J_sw,from_=0,to_=3.142,orient=HORIZONTAL,length=100,sliderlength=10)
    T2_unit = Label(J_sw,text=("rad/s"),font=(5))

    D3_velo = Label(J_sw,text=("D3* = "),font=(5)) 
    D3_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100)
    D3_unit = Label(J_sw,text=("cm/s"),font=(5))

    T1_velo.grid(row=0,column=0)
    T1_slider.grid(row=0,column=1)
    T1_unit.grid(row=0,column=2)

    T2_velo.grid(row=1,column=0)
    T2_slider.grid(row=1,column=1)
    T2_unit.grid(row=1,column=2)

    D3_velo.grid(row=2,column=0)
    D3_slider.grid(row=2,column=1)
    D3_unit.grid(row=2,column=2)

# Jacobian Entries and Labels
    x_velo = Label(J_sw,text=("x* = "),font=(5)) 
    x_entry = Entry(J_sw,width=10,font=(10))
    x_unit = Label(J_sw,text=("cm/s"),font=(5))
    x_velo.grid(row=3,column=0)
    x_entry.grid(row=3,column=1)
    x_unit.grid(row=3,column=2)

    y_velo = Label(J_sw,text=("y* = "),font=(5)) 
    y_entry = Entry(J_sw,width=10,font=(10))
    y_unit = Label(J_sw,text=("cm/s"),font=(5))
    y_velo.grid(row=4,column=0)
    y_entry.grid(row=4,column=1)
    y_unit.grid(row=4,column=2)

    z_velo = Label(J_sw,text=("z* = "),font=(5)) 
    z_entry = Entry(J_sw,width=10,font=(10))
    z_unit = Label(J_sw,text=("cm/s"),font=(5))
    z_velo.grid(row=5,column=0)
    z_entry.grid(row=5,column=1)
    z_unit.grid(row=5,column=2)

    ωx_velo = Label(J_sw,text=("ωx = "),font=(5)) 
    ωx_entry = Entry(J_sw,width=10,font=(10))
    ωx_unit = Label(J_sw,text=("rad/s"),font=(5))
    ωx_velo.grid(row=6,column=0)
    ωx_entry.grid(row=6,column=1)
    ωx_unit.grid(row=6,column=2)

    ωy_velo = Label(J_sw,text=("ωy = "),font=(5)) 
    ωy_entry = Entry(J_sw,width=10,font=(10))
    ωy_unit = Label(J_sw,text=("rad/s"),font=(5))
    ωy_velo.grid(row=7,column=0)
    ωy_entry.grid(row=7,column=1)
    ωy_unit.grid(row=7,column=2)

    ωz_velo = Label(J_sw,text=("ωz = "),font=(5)) 
    ωz_entry = Entry(J_sw,width=10,font=(10))
    ωz_unit = Label(J_sw,text=("rad/s"),font=(5))
    ωz_velo.grid(row=8,column=0)
    ωz_entry.grid(row=8,column=1)
    ωz_unit.grid(row=8,column=2)

    # Update Button
    update_but = Button(J_sw,text="Update",bg="green",fg="white",command=update_velo)
    update_but.grid(row=9,column=0)

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

    SCARA.teach (q=(0,0,0,0,0,0))

    # plot scale
    x1 = -0.8
    x2 = 0.8
    y1 = -0.8
    y2 = 0.8
    z1 = 0
    z2 = 0.8

    # Plot command
    SCARA.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def i_k():
    #link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())
    a5 = float(a5_E.get())

    #Position Vector in cm
    x0_3 = float(X_E.get())
    y0_3 = float(Y_E.get())
    z0_3 = float(Z_E.get())


    # Inverse Kinematic Solutions using Graphical Method

    # Solution 1
    phi2 = np.arctan(y0_3/x0_3)
    phi2 = phi2*180/np.pi

    # Solution 2
    r1 = np.sqrt((y0_3**2)+(x0_3**2))

    # Solution 3
    phi1 = np.arccos((a4**2-r1**2-a2**2)/(-2*r1*a2))
    phi1 = phi1*180/np.pi

    # Solution 4
    th1 = phi2 - phi1

    # solution 5
    phi3 = np.arccos((r1**2-a2**2-a4**2)/(-2*a2*a4))
    phi3 = phi3*180/np.pi

    # Solution 6
    th2 = 180 - phi3

    # Solution 7
    d3 =  a1 + a3 - a5 -z0_3  

    #Multiply the matrices

    t1_E.delete(0,END)
    t1_E.insert(0,np.around(th1,3))

    t2_E.delete(0,END)
    t2_E.insert(0,np.around(th2,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(d3,3))

def p_t():
    a1 = 3
    a2 = 3
    a3 = 3
    a4 = 3
    a5 = 3
    SCARA = DHRobot([
        PrismaticDH(0,0,0,0,qlim=[0,0]),
        RevoluteDH(a1,0,(0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a2,0,0,qlim=[0,0]),
        RevoluteDH(a3,0,(180/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a4,0,0,qlim=[0,0]),
        PrismaticDH(0,(0/180.0)*np.pi,(0/180.0)*np.pi,a5,qlim=[0,0])
    ], name = "SCARA")
    
    q0 = np.array([0,0,0,0,0,0])    
    q1 = np.array([0,
                    np.pi/4,
                   0,
                   4.071,
                   0,
                   1])
    q2 = np.array([0,
                   2,
                   0,
                   2,
                   0,
                   2])
    q3 = np.array([0,
                    -.172*np.pi,
                    0,
                   5,
                   0,
                   2.831])
    q4 = np.array([ 0,
                    -0.1476*np.pi,
                    0,
                   8,
                   0,
                   1.18])
    
    traj1 = rtb.jtraj(q0,q1,20)
    traj2 = rtb.jtraj(q1,q2,20)
    traj3 = rtb.jtraj(q2,q3,20)
    traj4 = rtb.jtraj(q3,q4,20)

    x1 = -15
    x2 = 15
    y1 = -15
    y2 = 15
    z1 = 0.0
    z2 = 15 
    
    SCARA.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj4.q,limits=[x1,x2,y1,y2,z1,z2],block=True)

def weld():
    a1 = 3
    a2 = 3
    a3 = 3
    a4 = 3
    a5 = 3
    SCARA = DHRobot([
        PrismaticDH(0,0,0,0,qlim=[0,0]),
        RevoluteDH(a1,0,(0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a2,0,0,qlim=[0,0]),
        RevoluteDH(a3,0,(180/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,a4,0,0,qlim=[0,0]),
        PrismaticDH(0,(0/180.0)*np.pi,(0/180.0)*np.pi,a5,qlim=[0,0])
    ], name = "SCARA")

    q0 = np.array([0,0,0,0,0,0])  
    q1 = np.array([0,
                    0,
                   4,
                   0,
                   2.831,
                   2])
    q2 = np.array([0,
                    np.pi/2,
                    0,
                   4,
                   0,
                   2.831])
    q3 = np.array([0,
                    1.5,
                    0,
                   3,
                   0,
                   2.831])
    q4 = np.array([0,
                    (3*np.pi)/2,
                    0,
                   2,
                   0,
                   2.831])
    q5 = np.array([0,
                    2*np.pi,
                    0,
                   1.5,
                   0,
                   2.831])
    q6 = np.array([0,
                    2*np.pi,
                   0,
                   0,
                   0,
                   0])
    
   
    
    traj1 = rtb.jtraj(q0,q1,20)
    traj2 = rtb.jtraj(q1,q2,20)
    traj3 = rtb.jtraj(q2,q3,20)
    traj4 = rtb.jtraj(q3,q4,20)
    traj5 = rtb.jtraj(q4,q5,20)
    traj6 = rtb.jtraj(q5,q6,20)

    x1 = -15
    x2 = 15
    y1 = -15
    y2 = 15
    z1 = 0.0
    z2 = 15 


    SCARA.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARAl.plot(traj4.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj5.q,limits=[x1,x2,y1,y2,z1,z2])
    SCARA.plot(traj6.q,limits=[x1,x2,y1,y2,z1,z2],block=True)  



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
    
    SCARA.teach (q=(0,0,0,0,0,0))

    # plot scale
    x1 = -0.8
    x2 = 0.8
    y1 = -0.8
    y2 = 0.8
    z1 = 0
    z2 = 0.8

    # Plot command
    SCARA.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)




##Frane
FI = LabelFrame(gui,text="Link Lenghts and joint variable", font=(5))
FI.grid(row=0,column=0)

#Link lenghts 
a1 = Label(FI,text=("a1 = "),font=(10))
a1_E = Entry(FI,width=5,font=(10))
cm1 = Label(FI,text="cm",font=(10))

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2 = Label(FI,text=("a2 = "),font=(10))
a2_E = Entry(FI,width=5,font=(10))
cm2 = Label(FI,text="cm",font=(10))

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3 = Label(FI,text=("a3 = "),font=(10))
a3_E = Entry(FI,width=5,font=(10))
cm3 = Label(FI,text="cm",font=(10))

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

a4 = Label(FI,text=("a4 = "),font=(10))
a4_E = Entry(FI,width=5,font=(10))
cm4 = Label(FI,text="cm",font=(10))

a4.grid(row=3,column=0)
a4_E.grid(row=3,column=1)
cm4.grid(row=3,column=2)

a5 = Label(FI,text=("a5 = "),font=(10))
a5_E = Entry(FI,width=5,font=(10))
cm5 = Label(FI,text="cm",font=(10))

a5.grid(row=4,column=0)
a5_E.grid(row=4,column=1)
cm5.grid(row=4,column=2)

## D

t1 = Label(FI,text=("t1 = "),font=(10))
t1_E = Entry(FI,width=5,font=(10))
deg1 = Label(FI,text="deg",font=(10))

t1.grid(row=0,column=3)
t1_E.grid(row=0,column=4)
deg1.grid(row=0,column=5)

t2 = Label(FI,text=("t2 = "),font=(10))
t2_E = Entry(FI,width=5,font=(10))
deg2 = Label(FI,text="deg",font=(10))

t2.grid(row=1,column=3)
t2_E.grid(row=1,column=4)
deg2.grid(row=1,column=5)

d3 = Label(FI,text=("d3 = "),font=(10))
d3_E = Entry(FI,width=5,font=(10))
cm6 = Label(FI,text="cm",font=(10))

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm6.grid(row=2,column=5)

BF = LabelFrame(gui,text="Forward and Inverse Kinematics",font=(5))
BF.grid(row=1,column=0)

#Buttons
FK = Button(BF,text="Forward",font=(10),bg="black",fg="pink",command=f_k)
rst = Button(BF,text="Reset",font=(10),bg="black",fg="pink",command=reset)
IK = Button(BF,text="Inverse",font=(10),bg="black",fg="pink",command=i_k)

FK.grid(row=0,column=0)
rst.grid(row=0,column=1)
IK.grid(row=0,column=2)


PV = LabelFrame(gui,text="Position Vector",font=(5))
PV.grid(row=2,column=0)

X = Label(PV,text=("x = "),font=(10))
X_E = Entry(PV,width=5,font=(10))
cm8 = Label(PV,text="cm",font=(10))

Y = Label(PV,text=("y = "),font=(10))
Y_E = Entry(PV,width=5,font=(10))
cm9 = Label(PV,text="cm",font=(10))

Z = Label(PV,text=("z = "),font=(10))
Z_E = Entry(PV,width=5,font=(10))
cm10 = Label(PV,text="cm",font=(10))

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm8.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm9.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm10.grid(row=2,column=2)

# insert image
img = PhotoImage(file= "scara.png")
img = img.subsample(5,5)
PI = Label(gui,image=img)
PI.grid(row=3,column=0)

#Path and Trajectory
PT = LabelFrame(gui,text="Path and Trajectory Planning")
PT.grid(row=4,column=0)

#Buttons for PT
PnP = Button(PT,text="Pick and Place",font=(10),bg="black", fg="pink",command=p_t)
Weld = Button(PT,text="Welding",font=(10),bg="hot pink", fg="pink",command=weld)

PnP.grid(row=0,column=0,padx=(40,0))
Weld.grid(row=0,column=1,padx=(0,40))

gui.mainloop()