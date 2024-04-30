#!/usr/bin/env python3


import sympy as sp
def get_trajectory(robot):
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