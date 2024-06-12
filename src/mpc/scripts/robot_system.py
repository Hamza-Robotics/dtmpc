#!/usr/bin/env python3


import sympy as sp
def get_trajectory(robot):
    t=sp.symbols('t')
    theta=sp.pi*t/70
    if robot=='robot1':

     
        k=0.5
        l=1
        time_delay=0
        x=1.2*sp.sin(theta-time_delay)
        y=1.2*sp.cos(theta-time_delay)
        xd=sp.diff(x,t)
        yd=sp.diff(y,t)
        
        x = sp.lambdify(t, x)
        y = sp.lambdify(t, y)
        xd = sp.lambdify(t, xd)
        yd = sp.lambdify(t, yd)
        

        return x,y ,xd,yd
    if robot=='robot2':
        
        k=0.
        l=0
        time_delay=0
        x=3*sp.sin(theta-time_delay)+l*sp.sin(theta-time_delay)
        y=3*sp.cos(theta-time_delay)+l*sp.cos(theta-time_delay)
        xd=sp.diff(x,t)
        yd=sp.diff(y,t)
        
        x = sp.lambdify(t, x)
        y = sp.lambdify(t, y)
        xd = sp.lambdify(t, xd)
        yd = sp.lambdify(t, yd)
        

        return x,y ,xd,yd
    
    if robot=='robot3':
        
      
        k=0.2
        l=0
        time_delay=0
        x=3*sp.sin(theta-time_delay)+l*sp.sin(theta-time_delay)
        y=3*sp.cos(theta-time_delay)+l*sp.cos(theta-time_delay)
        xd=sp.diff(x,t)
        yd=sp.diff(y,t)
        
        x = sp.lambdify(t, x)
        y = sp.lambdify(t, y)
        xd = sp.lambdify(t, xd)
        yd = sp.lambdify(t, yd)
        

        return x,y ,xd,yd
    

if __name__ == "__main__":
        # Your code here
    import casadi as ca
    class Robot():
        def __init__(self,name, neighborhood,c_range,radius,leader_node):
            self.name=name
            self.neighborhood=neighborhood
            self.c_range=c_range
            self.radius=radius
            self.leader_node=leader_node
            
        def communication_constraint(self,position,neighbor_position):
            if self.leader_node==False:
                x_robot1=position[0]
                y_robot1=position[1]

                x_robot2=neighbor_position[0]
                y_robot2=neighbor_position[1]
                c_range2=neighbor_position[2]
                distance=ca.sqrt((x_robot1-x_robot2)**2+(y_robot1-y_robot2)**2+0.0001)-(self.c_range+c_range2)
                return distance
        

        
        
        def get_trajectory(self,robot):
            t=sp.symbols('t')
            theta=sp.pi*t/50
            if robot=='robot1':

            
                k=0.5
                l=0.5
                x=3*sp.sin(theta)
                y=3*sp.cos(theta)
                xd=sp.diff(x,t)
                yd=sp.diff(y,t)
                
                x = sp.lambdify(t, x)
                y = sp.lambdify(t, y)
                xd = sp.lambdify(t, xd)
                yd = sp.lambdify(t, yd)
                

                return x,y ,xd,yd
            if robot=='robot2':
                
                k=0.
                l=0.5
                x=3*sp.sin(theta)+l*sp.sin(theta)
                y=3*sp.cos(theta)+l*sp.cos(theta)
                xd=sp.diff(x,t)
                yd=sp.diff(y,t)
                
                x = sp.lambdify(t, x)
                y = sp.lambdify(t, y)
                xd = sp.lambdify(t, xd)
                yd = sp.lambdify(t, yd)
                

                return x,y ,xd,yd
            
            if robot=='robot3':
                
            
                k=0.2
                l=0.5
                x=3*sp.sin(theta)-l*sp.sin(theta)
                y=3*sp.cos(theta)-l*sp.cos(theta)
                xd=sp.diff(x,t)
                yd=sp.diff(y,t)
                
                x = sp.lambdify(t, x)
                y = sp.lambdify(t, y)
                xd = sp.lambdify(t, xd)
                yd = sp.lambdify(t, yd)
                

                return x,y ,xd,yd
        
        
    robot1=Robot('robot1',[],1,0.1,True)
    robot2=Robot('robot2',['robot1'],1,0.1,False)
    robot3=Robot('robot3',['robot1'],1,0.1,False)


    class System():
        def __init__(self,robot_hierarchy):
            self.robot_hierarchy=robot_hierarchy
        def get_robot_and_hierarchy(self,robot_name):
            for index, robot in enumerate(self.robot_hierarchy):
                if robot.name == robot_name:
                    return robot,index