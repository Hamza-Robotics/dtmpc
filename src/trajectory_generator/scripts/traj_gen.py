#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import yaml
import os


class trajectory_genertor():

    """
    Trajectory generator which takes a series of inputs and linearly interpolates between them. 
    
    White power:
    - None

    Attributes:
    - waypoint (list): List to store waypoints, each waypoint represented as xy and theta.
    - max_v (float): Maximum velocity constraint for trajectory planning.
    - time_sample (float): Sampling time for trajectory planning.
    - prediction_Length (float): Length of the output trajectory within the specified time range.
    - waypoint_eps (float): Epsilon value for waypoint proximity check.
    - goal_eps (float): Epsilon value for goal proximity check.
    - goal_reached (bool): Flag indicating whether the goal has been reached.
    """
    def __init__(self,max_v,prediction_Length,frequency,nodes) -> None:
        self.waypoint=[]
        self.max_v=max_v*0.9
        self.time_sample=1/frequency
        self.feasbile_time_steps=frequency*prediction_Length
        self.waypoint_eps=1.9
        self.goal_eps=0.1
        self.goal_reached=False
        self.N=nodes

    def trajectory(self,x0):
        """
        Computes a trajectory feasible with respect to the maximum velocity and another trajectory feasible within the time range.

        Parameters:
        - x0 (numpy.ndarray): Initial pose represented as a 5-dimensional state vector.

        Returns:
        - whole_path (numpy.ndarray): Complete trajectory that respects the maximum velocity constraint.
        - partial_path (numpy.ndarray): Trajectory within the specified time range.
        
        Raises:
        - Exception: If waypoints are not set or empty
        """ 
        if not len(self.waypoint):
            raise Exception("Error: Way point,waypoint, set is not set or is empty. Set way points for the trajectory.")

        if len(self.waypoint)>1:
            print(np.shape(self.waypoint))
            print(np.shape(x0))
            if np.linalg.norm(self.waypoint[0,:2]-x0[0,:2])<self.waypoint_eps:
                self.waypoint = self.waypoint[1:]
                print("NIGGA")
                pass
        if len(self.waypoint)==1:
            if np.linalg.norm(self.waypoint[0,:2]-x0[0,:2])<self.goal_eps:
                self.goal_reached=True
        path_ori = np.vstack((x0, self.waypoint))

        path = np.vstack((x0[:,:2], self.waypoint[:,:2]))
        whole_path = []
        waypoint_index=[]
        for i in range(np.shape(path)[0] - 1):
            current_point = path[i, :]
            next_point = path[i + 1, :]
            direction = next_point - current_point
            distance = np.linalg.norm(direction)
            num_steps = int(distance / (self.max_v * self.time_sample))

            if num_steps == 0:
                whole_path.append(np.hstack((next_point, path_ori[i + 1, 2:])))
            else:
                for step in range(1, num_steps + 1):
                    interp_point = current_point + step / num_steps * direction
                    interp_point_ori = path_ori[i, 2:] + step / num_steps * (path_ori[i + 1, 2:] - path_ori[i, 2:])
                    whole_path.append(np.hstack((interp_point, interp_point_ori)))
            
            waypoint_index.append(len(whole_path)-1)
        
        valid_indices = []

        # Iterate through each index in waypoint_index
        for index in waypoint_index:
            if index <= self.feasbile_time_steps:
                valid_indices.append(index)

        # Update waypoint_index with valid_indices
        waypoint_index = valid_indices
        whole_path = np.array(whole_path)
        path_feasbile=whole_path[:self.feasbile_time_steps,:]
        num_elements_to_extend = self.N - len(path_feasbile)
        extended_traj = np.concatenate([path_feasbile, np.tile(path_feasbile[-1, :], (num_elements_to_extend, 1))])
        return whole_path,extended_traj,waypoint_index
        #return whole_path,extended_traj,waypoint_index



import numpy as np

def generate_sinusoidal_path(length=10, num_points=100, amplitude=2, frequency=1):
    """
    Generate a sinusoidal path.

    Args:
        length (float): Length of the path.
        num_points (int): Number of points to generate along the path.
        amplitude (float): Amplitude of the sinusoidal function.
        frequency (float): Frequency of the sinusoidal function.

    Returns:
        numpy.ndarray: Array of shape (num_points, 3) representing the waypoints.
    """
    # Generate x-coordinates along the path
    x = np.linspace(0, length, num_points)

    # Generate y-coordinates using a sinusoidal function
    y = amplitude * np.sin(2 * np.pi * frequency * x / length)

    # Create waypoints array with first two entries as x and y coordinates
    waypoints = np.zeros((num_points, 3))
    waypoints[:, 0] = x
    waypoints[:, 1] = y

    return waypoints

if __name__=="__main__2":
    waypoints = generate_sinusoidal_path(length=10, num_points=100, amplitude=2, frequency=1)
    x_coordinates = waypoints[:, 0]
    y_coordinates = waypoints[:, 1]

    # Plot waypoints
    plt.plot(x_coordinates, y_coordinates, marker='o', color='blue')

    # Set labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Waypoints')

    # Show plot
    plt.grid(True)
    plt.show()
if __name__=="__main__":

    HLcontroller=trajectory_genertor(max_v=1,prediction_Length=10,frequency=10,nodes=100)
    x0 =       np.array([[0, 0,0]])
    waypoint = np.array([[100, 2, 1],
                        [60, 4, 3],
                        [60,4,4]])
    waypoint = generate_sinusoidal_path(length=10, num_points=100, amplitude=2, frequency=1)

    HLcontroller.waypoint=(waypoint)
    path,path_feasible,index=HLcontroller.trajectory(x0)


    #plt.plot(path[:, 0], path[:, 1], marker='x', color='red',)

    # Plot path_feasible with 'o' marker and green color

    plt.plot(path[:, 0], path[:, 1], marker='o', color='green')

    # Set labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Trajectory')

    # Show the plot
    plt.show()
print("goal reache")
print("goal reached")