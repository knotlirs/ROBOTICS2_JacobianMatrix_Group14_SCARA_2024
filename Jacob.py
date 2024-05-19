import numpy as np
import sympy as sp

#link lengths in mm

a1 = float(input("a1 = "))

a2 = float(input("a2 = "))

a3 = float(input("a3 = "))

a4 = float(input("a4 = "))

a5 = float(input("a5 = "))

#joint variables: is mm if f, is degrees if theta

T1 = float(input("T1 = ")) #20 mm

T2 = float(input("T2 = ")) #30 deg

D3 = float(input("D3 = ")) #-90 deg

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

#print("H0_1= ")

#print(H0_1)

H1_2 = np.matrix(H1_2)

#print("H1_2= ")

#print(H1_2)

H2_3 = np.matrix(H2_3)

#print("H2_3= ")

#print(H2_3)

H0_2 = np.dot(H0_1,H1_2)

H0_3 = np.dot(H0_2,H2_3)

#print("H0_3= ")

#print(np.matrix(np.around(H0_3,3)))

## Jacobian Matrix

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
print("J = ")
print(np.around(J,3))

#4. Differential Equations
xp, yp, zp = sp.symbols('x* y* z*')
ωx, ωy, ωz = sp.symbols('ωx ωy ωz')
T1_p, T2_p, D3_p = sp.symbols('ϴ1* ϴ2* D3*')

q = [[T1_p],[T2_p],[D3_p]]

E = np.dot(J,q)
E = np.array(E)
print("E = ")
print(E)

xp = E[0,0]
yp = E[1,0]
zp = E[2,0]
ωx = E[3,0]
ωy = E[4,0]
ωz = E[5,0]

print("xp = ",xp)
print("yp = ",yp)
print("zp = ",zp)
print("ωx = ",ωx)
print("ωy = ",ωy)
print("ωz = ",ωz)

## Singularity
D_J = np.linalg.det(JM1)
print("D_J = ",D_J)

## Inverse Velocity
I_V = np.linalg.inv(JM1)
print("I_V = ",I_V)

# Force-Torque Analysis
F_T = np.transpose(JM1)
print("F_T = ")
print(F_T)

## Singularity
D_J = np.linalg.det(JM1)
print("D_J = ",D_J)

## Inverse Velocity
I_V = np.linalg.inv(JM1)
print("I_V = ",I_V)

# Force-Torque Analysis
F_T = np.transpose(JM1)
print("F_T = ")
print(F_T)
