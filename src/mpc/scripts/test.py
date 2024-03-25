import casadi as ca
import numpy as np

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel


####
# Define symbolic variables
model=AcadosModel()
v = ca.SX.sym('v')
th_d = ca.SX.sym('th_d')

# Define parameters variables
# State parameters
x = ca.SX.sym('x')
y = ca.SX.sym('y')
th = ca.SX.sym('th')
# Desired state parameters
x_d = ca.SX.sym('x_d')
y_d = ca.SX.sym('y_d')
# Desired velocity parameters
xdot_d = ca.SX.sym('xdot_d')
ydot_d = ca.SX.sym('ydot_d')


# Define error statements. 
e_x=x-x_d
e_y=y-y_d

e_d = ca.SX.sym('e_d')
e_o = ca.SX.sym('e_o')

# Define matrix using symbolic variables

# Concatenate symbolic variables into a vector
controls = ca.vertcat(v, th_d)
states = ca.vertcat(
                e_d,
                e_o)

paremeters = ca.vertcat(x, 
                y, 
                th,

                x_d,
                y_d,
                
                xdot_d,
                ydot_d,)    
eps=0.1
m11 = (e_x*ca.cos(th)+e_y*ca.sin(th))/(e_d+eps)**2
m12 = 0
m21 = -((e_y*ca.cos(th)-e_x*ca.sin(th))*(e_x*ca.cos(th)+e_y*ca.sin(th)))/(e_d+eps)**2
m22 = -((e_x/(e_d+eps))*ca.cos(th)+(e_y/(e_d+eps)**2)*ca.sin(th))

J = ca.vertcat(ca.horzcat(m11,m12),ca.horzcat(m21,m22))
# Concatenate matrices into a single matrix
#J = ca.vertcat(ca.horzcat(m11, m12), ca.horzcat(m21, m22))
e1 = -(e_x*xdot_d+e_y*ydot_d)/(e_d+eps)
e2 = (e_o*(xdot_d*e_x+ydot_d*e_y)+e_d*(ca.sin(th)-ca.cos(th)))/(e_d+eps)**2

# Calculate the result

E=ca.vertcat(e1, e2)
kin_eq = [ca.mtimes(J, controls)]

##

f = ca.Function('f', [states,paremeters, controls], [ca.vcat(kin_eq)], ['state','paremeters', 'control_input'], ['kin_eq'])
x_dot = ca.SX.sym('x_dot', len(kin_eq))
f_impl = x_dot - f(states,paremeters, controls)

model.f_expl_expr = f(states,paremeters, controls)
model.f_impl_expr = f_impl
model.x = states
model.xdot = x_dot
model.u = controls
model.p=paremeters
model.name = 'mobile_robot'

