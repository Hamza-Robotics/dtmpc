#from TNMPC import TNMPC as NMPC
from TNMPC import NMPC
import numpy as np
import matplotlib.pyplot as plt
import time
import time


def trajectory(Ts,N):
    traj=[]
    traj_dot=[]
    s=1
    for i in range(0,N):
        t=(Ts*i+time.time())*s
        
        x=3*np.sin(np.pi*t/25) +5
        y=3*np.cos(np.pi*t/25)+5

        xd=3*np.cos(np.pi*t/25)*np.pi/25
        yd=-3*np.sin(np.pi*t/25)*np.pi/25
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

def update_plot(x0, xd, prediction,solution,ed_serie,eo_serie,Ts):
    ed=solution[:,3]
    eo=solution[:,4]
    xp=prediction[:,0]
    yp=prediction[:,1]
    use_prediction = True  # Set to True if you want to plot the prediction instead of the solution
    plt.clf()  # Clear the previous plot
    if use_prediction:
        plt.subplot(2, 2, 1)  # First subplot for x position
        plt.plot(prediction[:,0], prediction[:,1], 'r-', label='Prediction')
        plt.plot(xd[:,0], xd[:,1], 'g-', label='Desired Trajectory')
        plt.plot(x0[0,0], x0[0,1], 'ro', label='Robot Position')  # Red dot for robot position
        plt.plot(xr[0,0], xr[0,1], 'go', label='Desired Position')  # Red dot for trajectory position
        plt.title('Position of robot and trajectory.')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-10, 10])  # Set x-axis limit
        plt.ylim([-10, 10])  # Set y-axis limit
        plt.legend()  # Add legend to the plot
        plt.grid(True)

    else:
        plt.subplot(2, 2, 1)  # First subplot for x position
        plt.plot(xp[:,0], xp[:,1], 'r-', label='Prediction')
        plt.plot(xd[:,0], xd[:,1], 'g-', label='Desired Trajectory')
        plt.plot(x0[0,0], x0[0,1], 'ro', label='Robot Position')  # Red dot for robot position
        plt.plot(xr[0,0], xr[0,1], 'go', label='Desired Position')  # Red dot for trajectory position
        plt.title('Position of robot and trajectory.')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-10, 10])  # Set x-axis limit
        plt.ylim([-10, 10])  # Set y-axis limit
        plt.legend()  # Add legend to the plot
        plt.grid(True)  
    print(Ts)
    plt.subplot(2, 2, 2)  # Second subplot for y position
    plt.plot(np.arange(0, len(ed)*Ts, Ts), ed, 'g-', label='ed')
    plt.plot(np.arange(0, len(eo)*Ts, Ts), eo, 'r-', label='eo')
    plt.title('Solution over Time')
    plt.xlabel('Time')
    plt.ylabel('ed and eo')
    plt.xlim([0, 10])  # Set x-axis limit
    plt.ylim([-6, 6])  # Set y-axis limit
    plt.legend()  # Add legend to the plot
    plt.grid(True)

    plt.subplot(2, 2, 3)  # Fourth subplot for diagnostics
    plt.plot(np.arange(0, len(ed_serie)*Ts, Ts), ed_serie, 'm-', label='ed')
    plt.plot(np.arange(0, len(eo_serie)*Ts, Ts), eo_serie, 'g-', label='eo')
    plt.xlim([0, 10])  # Set x-axis limit
    plt.ylim([-1, 5])  # Set y-axis limit
    plt.title('Diagnostics Over Time')
    plt.xlabel('Time')
    plt.ylabel('ed and eo evolution over time')
    plt.legend()  # Add legend to the plot
    plt.grid(True)


    if False:

        plt.subplot(2, 2, 3)  # Fourth subplot for diagnostics
        plt.plot(np.arange(0, len(ed_serie)*Ts, Ts), ed_serie, 'm-', label='ed')
        plt.plot(np.arange(0, len(eo_serie)*Ts, Ts), eo_serie, 'g-', label='eo')
        plt.xlim([0, 10])  # Set x-axis limit
        plt.ylim([-1, 5])  # Set y-axis limit
        plt.title('Diagnostics Over Time')
        plt.xlabel('Time')
        plt.ylabel('ed and eo evolution over time')
        plt.legend()  # Add legend to the plot
        plt.grid(True)

        plt.subplot(2, 2, 2)  # Fourth subplot for diagnostics
        plt.plot(range(len(diagnostics)), diagnostics, 'm-')
        plt.title('Diagnostics Over Time')
        plt.xlabel('Time')
        plt.ylabel('Diagnostics Value')
        plt.grid(True)

    plt.tight_layout()  # Adjust layout to prevent overlapping

    plt.pause(0.01)  # Pause to allow the plot to update



MPC = NMPC()

Ts=1/MPC.frequency


# Define the initial state
x0 = np.array([[2.5, 2.5, 0] ])
ed = np.zeros(MPC.N)
eo = np.zeros(MPC.N)
# Start the while loop
import time 
loop_hz = 20
loop_period = 1 / loop_hz
control_hz = 10
control_period = 1 / control_hz
start_time = time.time()


xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
for i in range(10):
    U,solution=MPC.controller(x0, xr, xd)
while True:
    # Get the latest trajectory data
    xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
    U,solution=MPC.controller(x0, xr, xd)
    x_prediction=prediction(x0,U)
    x0=propagate(x0,U[0,0],U[0,1],Ts)

    # Append 0.1 to the end of ed
    ed = np.append(ed, solution[0,3])
    ed = ed[1:]
    eo = np.append(eo, solution[0,4])
    eo = eo[1:]
    
    diagnostics = np.random.randn(len(xr))  # Assuming the same length as xr
    update_plot(x0, xr, x_prediction,solution,ed,eo, Ts)  # Update the plot