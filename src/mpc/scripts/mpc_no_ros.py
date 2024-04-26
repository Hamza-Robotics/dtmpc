from ATNMPC import NMPC
#from TNMPC import NMPC
#from NMPC  import NMPC
import numpy as np
import matplotlib.pyplot as plt
import time
import time
import threading

def trajectory(Ts,N):
    traj=[]
    traj_dot=[]
    s=1
    for i in range(0,N):
        t=(Ts*i+time.time())
        
        x=3*np.sin(np.pi*t/25) +5
        y=3*np.cos(np.pi*t/25)+5
        xd=3*np.cos(np.pi*t/25)*(np.pi/25)
        yd=-3*np.sin(np.pi*t/25)*(np.pi/25)
        traj.append([x,y])  
        traj_dot.append([xd,yd])
    return np.asanyarray(traj),np.asanyarray(traj_dot)

def propagate(x0,v,thd,Ts):
    x=x0[0,0]
    y=x0[0,1]
    th=x0[0,2]
    
    theta=th+thd*Ts
    x=x+v*np.cos(theta)*Ts
    y=y+v*np.sin(theta)*Ts

    return np.array([[x,y,theta]])

def prediction(x0,u):
    x=[]
    for i in range(len(u)):
        x.append(propagate(x0,u[i,0],u[i,1],Ts))
        x0=x[-1]


    return np.asanyarray(x).reshape(-1,3)

def update_plot(x0, xd, prediction,solution,ed_serie,eo_serie,obs,Ts):
    ed=solution[:,3]
    eo=solution[:,4]
    xp = solution[:,0]
    yp = solution[:,1]
    use_prediction = False  # Set to True if you want to plot the prediction instead of the solution
    plt.clf()  # Clear the previous plot
    if use_prediction:
        plt.subplot(2, 2, 1)  # First subplot for x position
        plt.plot(prediction[:,0], prediction[:,1], 'r-', label='Prediction')
        #plt.plot(xd[:,0], xd[:,1], 'g-', label='Desired Trajectory')
        plt.plot(x0[0,0], x0[0,1], 'ro', label='Robot Position')  # Red dot for robot position

        plt.plot(xr[0,0], xr[0,1], 'go', label='Desired Position')  # Green dot for trajectory position
        for ob in obs:
            circle = plt.Circle((ob[0], ob[1]), ob[2], color='r', fill=False)
            plt.gca().add_patch(circle)
            circle = plt.Circle((ob[0], ob[1]), ob[2], color='r', fill=False)
            plt.gca().add_patch(circle) 
        plt.title('Position of robot and trajectory.')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-10, 10])  # Set x-axis limit
        plt.ylim([-10, 10])  # Set y-axis limit
        plt.legend()  # Add legend to the plot
        plt.grid(True)

    else:
        plt.subplot(2, 2, 1)  # First subplot for x position
        plt.plot(xp, yp, 'r-', label='Prediction')
        plt.plot(xd[:,0], xd[:,1], 'g-', label='Desired Trajectory')
        plt.plot(x0[0,0], x0[0,1], 'ro', label='Robot Position')  # Red dot for robot position
        #plt.plot(xr[0,0], xr[0,1], 'go', label='Desired Position')  # Red dot for trajectory position
        for ob in obs:
            circle = plt.Circle((ob[0], ob[1]), ob[2], color='r', fill=False)
            plt.gca().add_patch(circle)

        plt.title('Position of robot and trajectory.')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-10, 10])  # Set x-axis limit
        plt.ylim([-10, 10])  # Set y-axis limit
        plt.legend()  # Add legend to the plot
        plt.grid(True)  
    plt.subplot(2, 2, 2)  # Second subplot for y position
    plt.plot(np.arange(0, len(ed)*Ts, Ts), ed, 'g-', label='ed')
    plt.plot(np.arange(0, len(eo)*Ts, Ts), eo, 'r-', label='eo')
    plt.title('Solution from MPC')
    plt.xlabel('Time')
    plt.ylabel('ed and eo')
    plt.xlim([0, 10])  # Set x-axis limit
    plt.ylim([-10, 10])  # Set y-axis limit
    plt.legend()  # Add legend to the plot
    plt.grid(True)
    plt.tight_layout()
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
    plt.tight_layout()

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







class Robot():
    def __init__(self):
        self.x=0
        self.y=0
        self.th0=0
        self.v=0
        self.th_d=0

        self.hz=50

    def propagate(self,v,th_d,hz):
        self.th0=self.th0+(th_d)*(1/(hz))
        self.x=self.x+np.cos(self.th0)*v*(1/(hz))
        self.y=self.y+np.sin(self.th0)*v*(1/(hz))
    
    def get_state(self):
        return np.array([[self.x,self.y,self.th0]])

MPC = NMPC()

Ts=1/MPC.frequency

robot1=Robot()

robot1.v=1
robot1.th_d=0.1


control_hz = 10
def control_loop():
    while True:
        xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
        obs=[[2,6,0.5]]
        U,solution=MPC.controller(robot1.get_state(), xr, xd,obs)
        robot1.v=U[0,0]
        robot1.th_d=U[0,1]
        
        ed = np.zeros(MPC.N)
        eo = np.zeros(MPC.N)
        # Append 0.1 to the end of ed
        ed = np.append(ed, solution[0,3])
        ed = ed[1:]
        eo = np.append(eo, solution[0,4])
        eo = eo[1:]

        
        update_plot(robot1.get_state(), xr, solution,solution,ed,eo,obs,Ts)  # Update the plot        
        time.sleep(1/control_hz)






system_loop_hz = 100
def system_loop():
    while True:
        robot1.propagate(robot1.v,robot1.th_d,system_loop_hz)
        print("20 Hz thread")
        time.sleep(1/system_loop_hz)

# Create and start the threads
thread1 = threading.Thread(target=control_loop)
thread2 = threading.Thread(target=system_loop)
thread1.start()
thread2.start()






while False:
    # Get the latest trajectory data
    xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
    obs=[[2,6,1.5]]
    xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values

    U,solution=MPC.controller(robot1.get_state(), xr, xd,obs)
    ed = np.zeros(MPC.N)
    eo = np.zeros(MPC.N)
    # Append 0.1 to the end of ed
    ed = np.append(ed, solution[0,3])
    ed = ed[1:]
    eo = np.append(eo, solution[0,4])
    eo = eo[1:]
    
    diagnostics = np.random.randn(len(xr))  # Assuming the same length as xr
    update_plot(robot1.get_state(), xr, solution,solution,ed,eo,obs,Ts)  # Update the plot
    

if False:

        

    MPC = NMPC()

    Ts=1/MPC.frequency


    # Define the initial state
    x0 = np.array([[2.5, 2.5, 0] ])
    ed = np.zeros(MPC.N)
    eo = np.zeros(MPC.N)
    # Start the while loop
    import time 
    import threading
    loop_hz = 20
    loop_period = 1 / loop_hz
    control_hz = 10
    control_period = 1 / control_hz
    start_time = time.time()

    obs=[[2,6,1.5]]

    xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
    # Define the initial state
    x0 = np.array([[xr[0,0]+0.3, xr[0,1]+0.1, 0] ])
    ed = np.zeros(MPC.N)
    eo = np.zeros(MPC.N)
    for i in range(10):
        U,solution=MPC.controller(x0, xr, xd,obs)
    while True:
        # Get the latest trajectory data
        xr, xd = trajectory(Ts,MPC.N)  # Assuming trajectory() returns x values
        U,solution=MPC.controller(x0, xr, xd,obs)
        x_prediction=prediction(x0,U)
        x0=propagate(x0,U[0,0],U[0,1],Ts)

        # Append 0.1 to the end of ed
        ed = np.append(ed, solution[0,3])
        ed = ed[1:]
        eo = np.append(eo, solution[0,4])
        eo = eo[1:]
        
        diagnostics = np.random.randn(len(xr))  # Assuming the same length as xr
        update_plot(x0, xr, x_prediction,solution,ed,eo,obs,Ts)  # Update the plot