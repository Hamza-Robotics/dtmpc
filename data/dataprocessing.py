
import pickle
import numpy as np
import matplotlib.pyplot as plt
start=35
N=60*200
# Unpickle the data
with open('scenario1/data_robot1_.pickle', 'rb') as file:
    data_robot1 = pickle.load(file)

    print(len(data_robot1)  )

    data_robot1 = data_robot1[start:N+start]


with open('scenario1/data_robot2_.pickle', 'rb') as file:
    data_robot2 = pickle.load(file)


    data_robot2 = data_robot2[start:N+start]


with open('scenario1/data_robot3_.pickle', 'rb') as file:
    data_robot3 = pickle.load(file)

    data_robot3 = data_robot3[start:N+start]


print(data_robot3[0])

print(len(data_robot1)  )
class datarobot():
    def __init__(self, data, id):
        self.data = data
        self.x = [point[0][0] for point in data]
        self.y = [point[0][1] for point in data]
        self.xd = [point[1][0] for point in data]
        self.yd = [point[1][1] for point in data]
        self.theta = [point[0][2] for point in data]
        self.u1= [point[2][0] for point in data]
        self.u2= [point[2][1] for point in data]
        self.time = [point[3] - min([point[3] for point in data]) for point in data]
        self.id = id
    def e_d(self, x, y, xd, yd):
        ex = x - xd
        ey = y - yd
        return (ex ** 2 + ey ** 2) ** 0.5
    def eo_d(self, x, y, theta, xd, yd):
        ex = x - xd
        ey = y - yd
        return ex * np.cos(theta) + ey * np.sin(theta)
    
    def ed_values(self):
        return [self.e_d(self.x[i], self.y[i], self.xd[i], self.yd[i]) for i in range(len(self.data))]
    def eo_values(self):
        return [self.eo_d(self.x[i], self.y[i], self.theta[i], self.xd[i], self.yd[i]) for i in range(len(self.data))]  
    

# Create an instance of the datarobot class
robot1 = datarobot(data_robot1,"robot1")
robot2 = datarobot(data_robot2,"robot2")
robot3 = datarobot(data_robot3,"robot3")

#

plot_traj = True
plot_control = False
plot_error = False  

if plot_error:

    plt.figure()
    plt.plot(robot1.ed_values(), label="robot1", color='r')
    plt.plot(robot2.ed_values(), label="robot2", color='g')
    plt.plot(robot3.ed_values(), label="robot3", color='b')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error distance")
    plt.grid()

    plt.figure()
    plt.plot(robot1.ed_values(), label="robot1", color='r')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error distance")
    plt.grid()
       
    plt.figure()
    plt.plot(robot2.ed_values(), label="robot2", color='g')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error distance")
    plt.grid()
       
    plt.figure()
    plt.plot(robot3.ed_values(), label="robot3", color='b')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error distance")
    plt.grid()
    plt.figure()
    plt.plot(robot1.eo_values(), label="robot1", color='r')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error orientation")
    plt.grid()
    plt.figure()
    plt.plot(robot2.eo_values(), label="robot2", color='g')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error orientation")
    plt.grid()
    plt.figure()
    plt.plot(robot3.eo_values(), label="robot3", color='b')
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("error [m]")
    plt.title("Error orientation")
    plt.grid()
    

if plot_control:
    plt.figure()
    plt.plot(robot1.time, robot1.u1, label="robot1", color='r')
    plt.xlabel("time")
    plt.ylabel("velocity [m/s]")
    plt.title("Control input velocity")
    plt.axhline(y=0.5, color='r', linestyle='--', label='velocity max')
    plt.axhline(y=-0.5, color='r', linestyle='--', label='velocity min')
    plt.legend()
    plt.figure()
    plt.plot(robot1.time, robot1.u2, label="robot1", color='r')
    plt.xlabel("time")
    plt.ylabel("angle velocity [rad/s]")
    plt.title("Control input angular velocity")
    plt.axhline(y=np.deg2rad(40), color='g', linestyle='--', label='angle velocity max')
    plt.axhline(y=np.deg2rad(-40), color='g', linestyle='--', label='angle velocity min')
    plt.legend()
    

    plt.figure()
    plt.plot(robot2.time, robot2.u1, label="robot2", color='g')
    plt.xlabel("time")
    plt.ylabel("velocity [m/s]")
    plt.title("Control input velocity")
    plt.axhline(y=0.5, color='r', linestyle='--', label='velocity max')
    plt.axhline(y=-0.5, color='r', linestyle='--', label='velocity min')
    plt.legend()
    plt.figure()
    plt.plot(robot2.time, robot2.u2, label="robot2", color='r')
    plt.xlabel("time")
    plt.ylabel("angle velocity [rad/s]")
    plt.title("Control input angular velocity")
    plt.axhline(y=np.deg2rad(40), color='g', linestyle='--', label='angle velocity max')
    plt.axhline(y=np.deg2rad(-40), color='g', linestyle='--', label='angle velocity min')
    plt.legend()
    

    plt.figure()
    plt.plot(robot3.time, robot3.u1, label="robot3", color='b')
    plt.xlabel("time")
    plt.ylabel("velocity [m/s]")
    plt.title("Control input velocity")
    plt.axhline(y=0.5, color='r', linestyle='--', label='velocity max')
    plt.axhline(y=-0.5, color='r', linestyle='--', label='velocity min')
    plt.legend()
    plt.figure()
    plt.plot(robot3.time, robot3.u2, label="robot3", color='r')
    plt.xlabel("time")
    plt.ylabel("angle velocity [rad/s]")
    plt.title("Control input angular velocity")
    plt.axhline(y=np.deg2rad(40), color='g', linestyle='--', label='angle velocity max')
    plt.axhline(y=np.deg2rad(-40), color='g', linestyle='--', label='angle velocity min')
    plt.legend()
    

if plot_traj:

    # Plot the grid
    plt.figure()
    
    for i in range(len(robot1.x)):
        if i % 100 == 0:
            circle1 = plt.Circle((robot1.x[i], robot1.y[i]), (1.6), linestyle='dashed', edgecolor='grey', facecolor='none')
            plt.gca().add_patch(circle1)
    plt.plot(robot1.x, robot1.y, label="robot1", color='r')
    plt.gca().add_patch(circle1)
    plt.plot(robot2.x, robot2.y, label="robot2", color='b')
    plt.plot(robot3.x, robot3.y, label="robot3", color='g')
    plt.axis('equal')
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Trajectory of the robots")
    plt.grid()


    circle1 = plt.Circle((1,4), 0.8/2, color='red')
    plt.gca().add_patch(circle1)
    circle2 = plt.Circle((-2,-2), 1.4/2, color='red')
    plt.gca().add_patch(circle2)
    circle3 = plt.Circle((-2,-4.3), 1.4/2, color='red')
    plt.gca().add_patch(circle3)
    






plt.show()
