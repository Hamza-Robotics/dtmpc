#from TNMPC import TNMPC as NMPC
from TNMPC import NMPC
import numpy as np
import matplotlib.pyplot as plt
import time

MPC = NMPC()
Ts=0.1
Ts=1/MPC.frequency


# Define the initial state
x0 = np.array([[0, 0, 0] ])

def trajectory(Ts,N):
    traj=[]
    traj_dot=[]
    s=1
    for i in range(0,N):
        t=(Ts*i+time.time())*s
        x=3*np.sin(np.pi*t/25)+2
        y=3*np.cos(np.pi*t/25)

        xd=3*np.cos(np.pi*t/25)*np.pi/25*s
        yd=-3*np.sin(np.pi*t/25)*np.pi/25*s
        traj.append([x,y])  
        traj_dot.append([xd,yd])
    return np.asanyarray(traj),np.asanyarray(traj_dot)

def propagate(x0,v,thd,Ts):
    x=x0[0,0]
    y=x0[0,1]
    th=x0[0,2]

    x=x+v*np.cos(th)*Ts
    y=y+v*np.sin(th)*Ts
    theta=th+thd*Ts
    return np.array([[x,y,theta]])

def prediction(x0,u):
    x=[]
    for i in range(len(u)):
        x.append(propagate(x0,u[i,0],u[i,1],Ts))
        x0=x[-1]


    return np.asanyarray(x).reshape(-1,3)

plt.ion()
fig, ax = plt.subplots()
point, = ax.plot([], [], 'bo')  # Blue point
robot, = ax.plot([], [], 'ro')  # Blue point
trajectory_line, = ax.plot([], [], 'b')  # Red line for the trajectory
robot_prediction, = ax.plot([], [], 'b')  # Red line for the trajectory

# Set x and y limits
ax.set_xlim(0, MPC.N*Ts)
ax.set_ylim(-5,5)

ax.set_xlim(-5, 5)
ax.set_ylim(-5,5)
# Set labels and title
ax.set_xlabel('Time')
ax.set_ylabel('X Value')
ax.set_title('Real-time X Value Plot')
U=np.array([[0,0]])
xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
U, S = MPC.controller(x0, xr, xd)


# Start the while loop
while True:
    # Get the latest trajectory data
    xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
    U,_=MPC.controller(x0, xr, xd)
    x_prediction=prediction(x0,U)
    x0=propagate(x0,U[0,0],U[0,1],Ts)
    robot.set_xdata(x0[0, 0])
    robot.set_ydata(x0[0, 1])
    # Update the plot with the latest x value
    point.set_xdata(xr[0, 0])  # Assuming you want to plot the x values against time
    point.set_ydata(xr[0, 1])  # Assuming you want to plot the first column of xr against time
    xd=xd[0,0]
    yd=xr[0,1]
    ex=x0[0,0]-xr[0,0]
    ey=x0[0,1]-xr[0,1]
    ed=np.sqrt(ex**2+ey**2)
    eo=(ey/ed)*np.cos(x0[0,2])-(ex/ed)*np.sin(x0[0,2])
    trajectory_line.set_xdata(xr[:, 0])
    trajectory_line.set_ydata(xr[:, 1])
    robot_prediction.set_xdata(_[:, 0])
    robot_prediction.set_ydata(_[:, 1])

    # Draw the plot
    plt.draw()
    plt.pause(0.01)  # Pause to allow the plot to update (adjust as needed)


length=len(S)
time_vector = np.arange(0, length*Ts, Ts)
plt.plot(time_vector, S[:, 0])
plt.plot(time_vector, S[:, 1])
plt.show(block=True)