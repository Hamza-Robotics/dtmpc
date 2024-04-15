from deriviation import generate_errorkin

def sympy2casadi(sympy_expr,sympy_var,casadi_var):
  import casadi
  assert casadi_var.is_vector()
  if casadi_var.shape[1]>1:
    casadi_var = casadi_var.T
  casadi_var = casadi.vertsplit(casadi_var)
  from sympy.utilities.lambdify import lambdify

  mapping = {'ImmutableDenseMatrix': casadi.blockcat,
             'MutableDenseMatrix': casadi.blockcat,
             'Abs':casadi.fabs
            }
  f = lambdify(sympy_var,sympy_expr,modules=[mapping, casadi])
  print(casadi_var)
  return f(*casadi_var)
  
  

import sympy 

import casadi as ca 

x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
xd = ca.SX.sym('xd')
yd = ca.SX.sym('yd')
theta_dot = ca.SX.sym('theta_dot')
v = ca.SX.sym('v')
xdot = ca.SX.sym('xdot')
ydot = ca.SX.sym('ydot')
xd_dot = ca.SX.sym('xd_dot')
yd_dot = ca.SX.sym('yd_dot')






ed,eo,state_sp=generate_errorkin()
state_ca=ca.vertcat(x,y,theta,xd,yd,xdot,ydot,xd_dot,yd_dot,theta_dot,v)

f = sympy2casadi(ed,state_sp,state_ca)

print(f)