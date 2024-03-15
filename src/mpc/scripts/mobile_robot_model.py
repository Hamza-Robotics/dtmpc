#!/usr/bin/env python
# coding=UTF-8

import numpy as np
import casadi as ca
from acados_template import AcadosModel

import os
import yaml


class MobileRobotModel(object):
    def __init__(self):

        current_directory=os.getcwd()
        with open(current_directory+'/src/mpc/config/mpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)

        model = AcadosModel() #  ca.types.SimpleNamespace()
        constraint = ca.types.SimpleNamespace()
        # control inputs
        v = ca.SX.sym('v')
        th_d = ca.SX.sym('th_d')
        controls = ca.vertcat(v, th_d)
        # n_controls = controls.size()[0]
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        th = ca.SX.sym('th')

        states = ca.vertcat(x, 
                            y, 
                            th)
        

        kin_eq = [(v)*ca.cos(th), 
               (v)*ca.sin(th),
               th_d 
                ]

        # function
        f = ca.Function('f', [states, controls], [ca.vcat(kin_eq)], ['state', 'control_input'], ['kin_eq'])
        # f_expl = ca.vcat(kin_eq)
        # acados model
        x_dot = ca.SX.sym('x_dot', len(kin_eq))
        f_impl = x_dot - f(states, controls)

        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
# set up parameters
        p = ca.vertcat([])
        model.p=p

        model.name = 'mobile_robot'

        # constraint
            

        
        constraint.v_max = (yamlfile['velocity_max'])
        constraint.v_min = (yamlfile['velocity_min'])


        constraint.th_d_max= np.deg2rad(yamlfile['wheel_angle_velocity_max'])
        constraint.th_d_min =np.deg2rad(yamlfile['wheel_angle_velocity_min'])
       
        
        constraint.expr = ca.vcat([v, th_d])

        self.model = model
        self.constraint = constraint