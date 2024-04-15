import sympy as sp
import sympy as sp


def generate_errorkin():
    x = sp.symbols('x', cls=sp.Function)
    y = sp.symbols('y', cls=sp.Function)
    xd = sp.symbols('xd', cls=sp.Function)
    yd = sp.symbols('yd', cls=sp.Function)
    theta = sp.symbols('theta', cls=sp.Function)
    t = sp.symbols('t')

    v = sp.symbols('v', cls=sp.Function)
    thd = sp.symbols('thd', cls=sp.Function)

    ex = x(t) - xd(t)
    ey = y(t) - yd(t)

    x = sp.symbols('x', cls=sp.Function)
    y = sp.symbols('y', cls=sp.Function)
    xd = sp.symbols('xd', cls=sp.Function)
    yd = sp.symbols('yd', cls=sp.Function)
    theta_dot = sp.symbols('theta', cls=sp.Function)
    t = sp.symbols('t')
    v = sp.symbols('v', cls=sp.Function)
    theta_dot = sp.symbols('thd', cls=sp.Function)
    ex = x(t) - xd(t)
    ey = y(t) - yd(t)

    xdot = sp.cos(theta(t)) * v(t)
    ydot = sp.sin(theta(t)) * v(t)
    xd_dot = sp.symbols('xd_dot', cls=sp.Function)  # Define xd_dot as a symbol
    yd_dot = sp.symbols('yd_dot', cls=sp.Function)  # Define yd_dot as a symbol
    ed = sp.sqrt(ex**2 + ey**2)

    ed_dot = sp.diff(ed, t)

    #ed_dot = ed_dot.subs(sp.Derivative(x(t), t), xdot)
    #ed_dot = ed_dot.subs(sp.Derivative(y(t), t), ydot)

    ed_dot = ed_dot.subs(sp.Derivative(yd(t), t), sp.symbols('yd_dot', cls=sp.Function)(t))
    ed_dot = ed_dot.subs(sp.Derivative(xd(t), t), sp.symbols('xd_dot', cls=sp.Function)(t))

    ed_dot = ed_dot.subs(sp.Derivative(y(t), t), sp.symbols('y_dot', cls=sp.Function)(t))
    ed_dot = ed_dot.subs(sp.Derivative(x(t), t), sp.symbols('x_dot', cls=sp.Function)(t))

    #ed_dot = ed_dot.subs(x(t) - xd(t), sp.symbols('e_x', cls=sp.Function)(t))
    #ed_dot = ed_dot.subs(y(t) - yd(t), sp.symbols('e_y', cls=sp.Function)(t))

    ed_dot = ed_dot.subs(sp.sqrt(ex**2 + ey**2), sp.symbols('ed', cls=sp.Function)(t))   
    ed_dot = ed_dot.subs(x(t) - xd(t), sp.symbols('e_x', cls=sp.Function)(t))
    ed_dot = ed_dot.subs(y(t) - yd(t), sp.symbols('e_y', cls=sp.Function)(t))
    ed_dot = sp.simplify(ed_dot)


    eo=(ey/ed)*sp.sin(theta(t))-(ex/ed)*sp.cos(theta(t))
    eo_dot = sp.diff(eo, t)
    eo_dot = eo_dot.subs(sp.Derivative(y(t), t), sp.symbols('y_dot', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(sp.Derivative(x(t), t), sp.symbols('x_dot', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(sp.Derivative(yd(t), t), sp.symbols('yd_dot', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(sp.Derivative(xd(t), t), sp.symbols('xd_dot', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(sp.sqrt(ex**2 + ey**2), sp.symbols('ed', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(x(t) - xd(t), sp.symbols('e_x', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(y(t) - yd(t), sp.symbols('e_y', cls=sp.Function)(t))
    eo_dot = eo_dot.subs(sp.Derivative(theta(t), t),sp.symbols('theta_dot', cls=sp.Function)(t))
    eo_dot = sp.simplify(eo_dot)    
    
    #state_sp=sp.Matrix([x(t),y(t),theta(t),xd(t),yd(t),xdot,ydot,xd_dot,yd_dot,theta_dot(t),v(t)])
    return ed_dot, eo_dot, #state_sp

import casadi as ca



ed,eo=generate_errorkin()
eo=(str(eo).replace("(t)", ""))
eo=(str(eo).replace("cos", "ca.cos"))
eo=(str(eo).replace("sin", "ca.sin"))
#print((eo))

ed=(str(ed).replace("(t)", ""))
ed=(str(ed).replace("cos", "ca.cos"))
ed=(str(ed).replace("sin", "ca.sin"))
print((ed))
