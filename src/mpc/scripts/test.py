import casadi as ca
import numpy as np

##
# Define symbolic variables
v = ca.SX.sym('v')
th_d = ca.SX.sym('th_d')

# Define symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
th = ca.SX.sym('th')
x_d = ca.SX.sym('x_d')
y_d = ca.SX.sym('y_d')
# Desired velocity
xdot_d = ca.SX.sym('xdot_d')
ydot_d = ca.SX.sym('ydot_d')
# Define error statements. 
e_x=x-x_d
e_y=y-y_d
e_d=ca.sqrt(e_x**2+e_y**2)
e_o=(e_y*ca.cos(th))/e_d-(e_x*ca.sin(th))/e_d
e_d = ca.SX.sym('e_d')
e_o = ca.SX.sym('e_o')
# Define matrix using symbolic variables

# Concatenate symbolic variables into a vector
controls = ca.vertcat(v, th_d)
states = ca.vertcat(x, 
                    y, 
                    th,

                    x_d,
                    y_d,
                    
                    xdot_d,
                    ydot_d,
                    e_d,
                    e_o)

m11 = (e_x*ca.cos(th)+e_y*ca.sin(th))/e_d
m12 = 0
m21 = ((e_y*ca.cos(th)-e_x*ca.sin(th))*(e_x*ca.cos(th)+e_y*ca.sin(th)))/e_d**2
m22 = (e_x*ca.cos(th)+e_y*ca.sin(th))/e_d

# Concatenate matrices into a single matrix
J = ca.vertcat(ca.horzcat(m11, m12), 
                    ca.horzcat(m21, m22))
e1 = -(e_x*xdot_d+e_y*ydot_d)/e_d
e2 = (e_o*(xdot_d*e_x+ydot_d*e_y)+e_d*(ca.sin(th)-ca.cos(th)))/e_d**2

# Calculate the result

E=ca.vertcat(e1, e2)
kin_eq = [([1]*(9-2)),ca.mtimes(J, controls) + E]

##

print(  kin_eq)
f = ca.Function('f', [states, controls], [ca.vcat(kin_eq)], ['state', 'control_input'], ['kin_eq'])
x_dot = ca.SX.sym('x_dot', len(kin_eq))

print(len(kin_eq))

f_impl = x_dot - f(states, controls)
