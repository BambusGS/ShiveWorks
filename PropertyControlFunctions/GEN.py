#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  5 17:32:12 2023

@author: bill
"""
# Import numpy and matplotlib
import csv
import numpy as np
import matplotlib.pyplot as plt

# Import python wrapped Fortran functions for calculation
import Test


#### Definition of Inputs####
# Spatial Parameters
a = 0
b = 2  # Total interval for spatial points
T0 = 0.0
Tf = 2.0  # Total interval for temporal points

# Geometry Parameters -- used in both FG CB or FG Lam
eps = (b-a)/2
tau = (Tf-T0)/2  # These are the spatial-temporal periods for CB
m1 = .5
n1 = .5  # These are the spatial-temporal volume fractions of the periods

# Functionally Graded(FG) smoothing properties in space and time -
# Larger values give more smoothing
smt_x = .1
smt_t = .2  # Spatial-Temporal Smoothing

V = .1  # Lamination velocity -- Only Used for Lamination

# Not used Currently
# c1=1.1; c2=2.3; # Values of Material Properties in material 1 and 2

###############################################################################
# Plotting Parameters
M = 400
N = 400  # Number of sample points along x and t for fine plot
###############################################################################


#### Creation of fine mesh for pcolor plotting evaluation and verification ####
x = np.linspace(a, b, M)
t = np.linspace(T0, Tf, N)
X, T = np.meshgrid(x, t)
xx = X.flatten(order='F')
tt = T.flatten(order='F')

#### Shive Machine- Set the number of spatial elements and the sample rate ####
N_SpatElem = 40  # Number of Spatial Elements on the Device
Mat1 = 0
Mat2 = 255  # Material Range from Material 1 to Material 2
t_SampPeri = 1/1000.  # Time Sample Period in Milliseconds
N_SampTime = int((Tf-T0)/t_SampPeri)  # Number of sample times
###############################################################################

# Evenly spaced array of spatial location of elements from a to b
x_SpatElem = np.linspace(a, b, N_SpatElem)
# Evenly spaced array of time values from T0 to Tf
t_SampTime = np.linspace(T0, Tf, N_SampTime)

# Creates a meshgrid array of elements to capture all of the possible locations in space-time
X_Samp, T_Samp = np.meshgrid(x_SpatElem, t_SampTime)
xx_Samp = X_Samp.flatten(order='F')
# Flattens the previously created meshgrid (in Fortran Order) to allow for faster calculation
tt_Samp = T_Samp.flatten(order='F')


# #### The following code names the functions from the fortran module as python functions
# g1=Test.testfunctions.psi_lam # Sharp Lamination
# g2=Test.testfunctions.psi_fg_lam # Functionally Graded Lamination
# g3=Test.testfunctions.psi_cbd # Sharp Checkoerboard
# g4=Test.testfunctions.psi_fg_cbd # Functionally Graded Checkerboard
# g5=Test.testfunctions.f_u # Piecewise Functionally Graded Checkerboard

# The following code is the selected function that we will be using in the project, the FG Checkerboard
# I am using the functionally graded checkerboard option here but this can be changed to any of the other
# functions found in the Fortran Module I have included the FG lamination commented out below

g = Test.testfunctions.psi_fg_cbd
# This function maps g onto the previously created arrays xx and tt
zz_g = np.array(list(map(lambda x, t: g(x, t, Mat1, Mat2, m1, n1, eps,
                tau, smt_x, smt_t), xx_Samp, tt_Samp)))  # FG CBD on Shive Devices
zz_fine = np.array(list(map(lambda x, t: g(x, t, Mat1, Mat2, m1,
                   n1, eps, tau, smt_x, smt_t), xx, tt)))  # FG CBD on Fine Mesh


# Uncomment the following lines to get the functionally graded lamination instead of CB
# g=Test.testfunctions.psi_fg_lam
# ## This function maps g onto the previously created arrays xx and tt
# zz_g=np.array(list(map(lambda x,t: g(x,t,Mat1,Mat2,m1,eps,V,smt_x),xx_Samp,tt_Samp))) #FG LAM on Shive Devices
# zz_fine=np.array(list(map(lambda x,t: g(x,t,Mat1,Mat2,m1,eps,V,smt_x),xx,tt))) #FG LAM on Fine Mesh

###############################################################################

# What follows is definition of one of the output objects. All are matrix
# arrays of size (N_SampTime,N_SpatElem)

# The values in Z_g are floats ranging from 0 (material 1) to 255 (material 2)
Z_g = zz_g.reshape(X_Samp.shape, order='F')
# The following is the integer conversion of the previous output object
Z_g_int = np.asarray(Z_g, dtype='int')

# The values in K_g range from 0 (material 1) to 255 (material 2)
# only in regions where the material is changing more than some tolerance
# and -100 in regions where the material properties are approximately constant

# The point of the array K_g is to identify regions of constant material properties
# using a maximum gradient value as a tolerance by identifying regions of constancy
# by flagging cell values whose gradient (rate of change) is very small.
GradTol = .1  # Maximum tolerance of gradient

K_g = np.zeros(Z_g.shape)  # Preallocating K_g array

indx_Const = np.where(np.abs(np.gradient(Z_g, axis=0)) <=
                      GradTol)  # Approximately constant cells
indx_Chang = np.where(np.abs(np.gradient(Z_g, axis=0)) >=
                      GradTol)  # Changing material property cells

K_g[indx_Chang] = Z_g[indx_Chang]
K_g[indx_Const] = -100


###############################################################################
# Creation of fine mesh arrays for plotting 2d pcolormesh
Z_fine = zz_fine.reshape(X.shape, order='F')
Z_fine_int = np.asarray(Z_fine, dtype='int')

###############################################################################
# The following code is for plotting and verification and can be commented
# out before running

# Plot of the 2D pcolor in space and time
fig6 = plt.figure(6)
ax6_1 = plt.axes()
plot6_1 = ax6_1.pcolormesh(X, T, Z_fine, zorder=0, shading='gouraud')
plt.colorbar(plot6_1)
ax6_1.set_xlabel(r'$z$')
ax6_1.set_ylabel(r'$t$')


# Plot of the 1D time
fig7 = plt.figure(7)
ax7_1 = plt.axes()
plot7_1 = ax7_1.plot(T_Samp[:], Z_g[:])
ax7_1.set_xlabel(r'$t$')
ax7_1.set_ylabel(r'$u$')


fig8 = plt.figure(8)
ax8_1 = plt.axes()
plot8_1 = ax8_1.pcolormesh(X, T, Z_fine_int)
plt.colorbar(plot8_1)
ax8_1.set_xlabel(r'$z$')
ax8_1.set_ylabel(r'$t$')


fig9 = plt.figure(9)
ax9_1 = plt.axes()
plot9_1 = ax9_1.plot(T_Samp[:], Z_g_int[:])
ax9_1.set_xlabel(r'$t$')
ax9_1.set_ylabel(r'$u$')
# plt.savefig("plot.png")


fig9 = plt.figure(10)
ax10_1 = plt.axes()
plot10_1 = ax10_1.plot(T_Samp[0:-1], np.diff(Z_g, axis=0), c='r')
plot10_2 = ax10_1.plot(T_Samp[:], np.gradient(Z_g, axis=0), c='b')

ax10_1.set_xlabel(r'$t$')
ax10_1.set_ylabel(r'$u$')


fig11 = plt.figure(11)
ax11_1 = plt.axes()

plot11_1 = ax11_1.scatter(T_Samp[:, :], K_g[:, :], s=1)
plot11_2 = ax11_1.plot(T_Samp[:, :], K_g[:, :])
plt.colorbar(plot11_1)
ax11_1.set_xlabel(r'$z$')
ax11_1.set_ylabel(r'$t$')


### Old Test Code ################

# # Initial creation of mesh for function evaluation
# x=np.linspace(a,b,M); t=np.linspace(T0,Tf,N)
# X,T=np.meshgrid(x,t);
# xx=X.flatten(order='F'); tt=T.flatten(order='F')

# ## Uncomment the following line to print out the list of functions in Test
# # print(Test.__doc__)

# ## The following code names the functions from the fortran module as python functions
# PSI_LAM=Test.testfunctions.psi_lam
# PSI_FG_LAM=Test.testfunctions.psi_fg_lam
# PSI_CBD=Test.testfunctions.psi_cbd
# PSI_FG_CBD=Test.testfunctions.psi_fg_cbd
# F_U=Test.testfunctions.f_u


# ## This function maps the lamination onto the arrays xx and tt
# zz_PSI_LAM=np.array(list(map(lambda x,t: PSI_LAM(x,t,c1,c2,m1,eps,V),xx,tt)))
# Z_PSI_LAM=zz_PSI_LAM.reshape(X.shape,order='F')


# ## This function maps the lamination onto the arrays xx and tt
# zz_PSI_FG_LAM=np.array(list(map(lambda x,t: PSI_FG_LAM(x,t,c1,c2,m1,eps,V,smt_x),xx,tt)))
# Z_PSI_FG_LAM=zz_PSI_FG_LAM.reshape(X.shape,order='F')


# ## This function maps the lamination onto the arrays xx and tt
# zz_PSI_CBD=np.array(list(map(lambda x,t: PSI_CBD(x,t,c1,c2,m1,n1,eps,tau),xx,tt)))
# Z_PSI_CBD=zz_PSI_CBD.reshape(X.shape,order='F')


# ## This function maps the lamination onto the arrays xx and tt
# zz_PSI_FG_CBD=np.array(list(map(lambda x,t: PSI_FG_CBD(x,t,c1,c2,m1,n1,eps,tau,smt_x,smt_t),xx,tt)))
# Z_PSI_FG_CBD=zz_PSI_FG_CBD.reshape(X.shape,order='F')


# ## This function maps the lamination onto the arrays xx and tt
# zz_F_U=np.array(list(map(lambda x,t: F_U(x,t,c1,c2,4),xx,tt)))
# Z_F_U=zz_F_U.reshape(X.shape,order='F')


# ## Plots for checking results of the functions
# fig1 = plt.figure(1)
# ax1_1 = plt.axes()
# plot1_1=ax1_1.pcolormesh(X,T,Z_PSI_LAM)
# plt.colorbar(plot1_1)


# fig2 = plt.figure(2)
# ax2_1 = plt.axes()
# plot2_1=ax2_1.pcolormesh(X,T,Z_PSI_FG_LAM)
# plt.colorbar(plot2_1)


# fig3 = plt.figure(3)
# ax3_1 = plt.axes()
# plot3_1=ax3_1.pcolor(X,T,Z_PSI_CBD)
# plt.colorbar(plot3_1)


# fig4 = plt.figure(4)
# ax4_1 = plt.axes()
# plot4_1=ax4_1.pcolor(X,T,Z_PSI_FG_CBD)
# plt.colorbar(plot4_1)


# fig5 = plt.figure(5)
# ax5_1 = plt.axes()
# plot5_1=ax5_1.pcolor(X,T,Z_F_U)
# plt.colorbar(plot5_1)


# ------------------#

# a function that takes the K_g array and creates a CVS file for the each segment in the format of "timestamp, material"


def makeCVSfiles():  # the array K_g stores the material values for each segment at each time step including time steps where the material is not changing
    folderName = "Actuation_data"
    total_number_segments = N_SpatElem

    try:
        # check that the total runtime is not greater than 65535 milliseconds
        if len(t_SampTime) > 65535:
            raise ValueError("Total runtime must be less than 65535 ms")

        for segment_number in range(total_number_segments):
            segmentFilePath = folderName + '/' + \
                str(segment_number + 1) + ".csv"

            # generate the 2D array that hold time and material value [timestamp, material]
            #      timestamps are stored in t_SampTime linearly as milliseconds
            #      K_g[x][y] stores value for each timestamp as [x], and segment number zero-indexed at [y]
            segmentArray = []

            # go through each timestamp that has been generated
            for time_index in range(len(t_SampTime)):
                # convert to milliseconds as integer value
                timestamp = round(t_SampTime[time_index] * 1000)
                # K_g[x][y] stores value for each timestamp as [x], and segment number zero-indexed at [y]
                material_at_timestamp = round(K_g[time_index][segment_number])
                segmentArray.append([timestamp, material_at_timestamp])
            # print(segmentArray[:20])

            with open(segmentFilePath, 'w', newline='') as file:
                writer = csv.writer(file, delimiter=',')
                for row in segmentArray:
                    writer.writerow(row)
        return True

    except Exception as e:
        print("Failed to save segments ID list: {}".format(e))
        return False


print("Generating actuation data successful?: {}".format(makeCVSfiles()))
