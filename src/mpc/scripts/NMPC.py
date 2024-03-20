#!/usr/bin/env python3
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import casadi as ca
from mobile_robot_model import MobileRobotModel
import yaml
import numpy as np
import scipy
import os

class  NMPC():

    def __init__(self):
        with open('src/mpc/config/mpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        self.__QP_solver = yamlfile['QP_solver']
        self.frequency=yamlfile['Prediction_Frequency']
        self.__prediction_length = yamlfile['Prediction_Length']
        self.update_frequency=yamlfile['Update_Frequency']
        self.__robot_model=MobileRobotModel()

        self.__Tf = self.__prediction_length
        self.__N=int(self.__prediction_length*self.frequency)
        self.N=int(self.__prediction_length*self.frequency)
        self.numberofobs=yamlfile['Number_obstacles']
        self.cost_map=0.0
        self.__model=self.__robot_model.model # Model
        self.__ocp = AcadosOcp()
        self.__ocp.model = self.__model #Define Model
        self.__mconstraint=self.__robot_model.model
        # Initialize Parameters.
        self.__n_params = self.__model.p.size()[0]
        self.max_v=self.__robot_model.constraint.v_max
        self.min_v=self.__robot_model.constraint.v_min
        self.max_th_d=self.__robot_model.constraint.th_d_max
        self.min_th_d=self.__robot_model.constraint.th_d_min
        self.prediction_length=self.__prediction_length
        self.__ocp.dims.np = self.__n_params
        self.__ocp.parameter_values = np.zeros(self.__n_params)
        self.__ocp.dims.N = self.__N

        # Vehicle parameters
        self.__nlp_solver_type=yamlfile['nlp_solver_type']
        self.__Q_matrix_e=yamlfile['Q_matrix_e']
        self.__Q_matrix=yamlfile['Q_matrix']        
        self.__Q_matrix_waypoint=yamlfile['Q_matrix_waypoint']  
        self.__R_matrix=yamlfile['R_matrix']
        self.__print_status=yamlfile['print_acados_status']
        self.__nlp_solver_max_iter=yamlfile['nlp_solver_max_iter']
        self.__backtowheel = 0 # [m]
        self.__integrator_type=yamlfile['integrator_type']
        self.__L2_Norm=yamlfile['L2_Norm']
        self.__L1_Norm=yamlfile['L1_Norm']
        self.__to_plotsystem=yamlfile['to_plotsystem']
        

        self.__obstacle=0




        self.__linear_cost_function()
        self.__constraints()
        self.__solver_compiler()
    
    def __linear_cost_function(self):
        
        # cost type
        self.__nx = self.__model.x.size()[0] 
        self.__nx = self.__nx #Number of states
        self.__nu = self.__model.u.size()[0]
        self.__ny = self.__nx + self.__nu
        self.__ny_e = self.__nx
        ### COST FUNCTION ##
        unscale=self.__N/self.__Tf
        self.__Q =  np.diag(self.__Q_matrix) # [x,y,x_d,y_d,th,th_d]
        self.__Q_e =  np.diag(self.__Q_matrix_e) # [x,y,x_d,y_d,th,th_d]


        self.__R = np.diag(self.__R_matrix)
        self.__ocp.cost.cost_type = 'LINEAR_LS'
        self.__ocp.cost.cost_type_e = 'LINEAR_LS'

        self.__ocp.cost.W = scipy.linalg.block_diag(self.__Q, self.__R)
        self.__ocp.cost.W_0 = scipy.linalg.block_diag(self.__Q, self.__R)


        self.__ocp.cost.W_e=  np.diag(self.__Q_matrix)# [x,y,x_d,y_d,th,th_d]
        self.__ocp.cost.Vx = np.zeros((self.__ny, self.__nx))
        self.__ocp.cost.Vx[:self.__nx, :self.__nx] = np.eye(self.__nx)

        self.__ocp.cost.Vx_0 = np.zeros((self.__ny, self.__nx))
        self.__ocp.cost.Vx_0[:self.__nx, :self.__nx] = np.eye(self.__nx)
        self.__ocp.cost.Vu = np.zeros((self.__ny, self.__nu))
        self.__ocp.cost.Vu[-self.__nu:, -self.__nu:] = np.eye(self.__nu)

        self.__ocp.cost.Vu_0 = np.zeros((self.__ny, self.__nu))
        self.__ocp.cost.Vu_0[-self.__nu:, -self.__nu:] = np.eye(self.__nu)
        self.__ocp.cost.Vx_e = np.eye(self.__nx)

        self.__ocp.cost.yref = np.zeros((self.__ny,))
        self.__ocp.cost.yref_e = np.zeros((self.__ny_e,))
    

        print(np.shape(self.__ocp.cost.Vx_0))
        print(np.shape(self.__ocp.cost.Vu_0))
        print(self.__ocp.cost.W_0.shape[0])

    def __constraints(self):
        self.__ns_e=0
        self.__ns_0=0
        self.__ns_i=0
        x_alg=self.__model.x[0]
        y_alg=self.__model.x[1]
        th_alg=self.__model.x[2]
        self.__ocp.constraints.lbu = np.array([self.min_v, (self.min_th_d)])
        self.__ocp.constraints.ubu = np.array([self.max_v, (self.max_th_d)])
        self.__ocp.constraints.idxbu = np.array([0, 1])
        self.__ocp.constraints.lsbu = np.zeros(2)
        self.__ocp.constraints.usbu = np.zeros(2)
        self.__ocp.constraints.idxsbu = np.array([0, 1])
        self.__ns_i+=2
        self.__ns_0+=2








        if self.__obstacle:

            for i in range(0,self.__n_params,3):

                x_p=self.__model.p[i+0]
                y_p=self.__model.p[i+1]
                r_p=self.__model.p[i+2]
                capra_radius=np.sqrt((self.__robot_length/2)**2 + (self.__robot_width/2)**2)+self.__wheel_len/2
                con_h.append(((x_alg-x_p)**2 + (y_alg-y_p)**2)-((r_p+capra_radius+self.__safetydistance)**2))

                

                x_front_wheel=x_alg+(self.__robot_length/2+self.__wheel_len/2+self.__wheel_forward/2)*np.cos(th0_alg)
                y_front_wheel=y_alg+(self.__robot_length/2+self.__wheel_len/2+self.__wheel_forward/2)*np.sin(th0_alg)               
                con_h.append(((x_front_wheel-x_p)**2 + (y_front_wheel-y_p)**2)-((self.__safetydistance+r_p+self.__wheel_len/2)**2))

                x_back_wheel=x_alg-(self.__robot_length/2+self.__wheel_len/2+self.__wheel_forward/2)*np.cos(th0_alg)
                y_back_wheel=y_alg-(self.__robot_length/2+self.__wheel_len/2+self.__wheel_forward/2)*np.sin(th0_alg)  
                con_h.append(((x_back_wheel-x_p)**2 + (y_back_wheel-y_p)**2)-((r_p+self.__wheel_len/2)**2))


                l1=self.__trailer_link_length
                tpx=x_alg-l1*np.cos(th1_alg)
                tpy=y_alg-l1*np.sin(th1_alg)
                trailer_radius=np.sqrt((self.__trailer_length/2)**2 + (self.__trailer_width/2)**2)+self.__wheel_len/2
                con_h.append(( (tpx-x_p)**2 + (tpy-y_p)**2)-((r_p+trailer_radius+self.__safetydistance)**2) )

            con_h_vcat=vcat(con_h)
            self.__ocp.model.con_h_expr =con_h_vcat
            self.__ocp.constraints.lh = (0*    np.ones((len(con_h),)))
            self.__ocp.constraints.uh = (1e3 * np.ones((len(con_h),)))
            self.__ocp.constraints.lsh = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh = np.array(range(len(con_h)))    # Jsh
            self.__ns_i+=len(con_h)

            con_h_vcat=vcat(con_h)
            self.__ocp.model.con_h_expr_e =con_h_vcat
            self.__ocp.constraints.lh_e = (0*    np.ones((len(con_h),)))
            self.__ocp.constraints.uh_e = (1e3 * np.ones((len(con_h),)))
            self.__ocp.constraints.lsh_e = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush_e = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh_e = np.array(range(len(con_h)))    
            self.__ns_e+=len(con_h)

            self.__ocp.model.con_h_expr_0 =con_h_vcat
            self.__ocp.constraints.lh_0 = (0*    np.ones((len(con_h),)))
            self.__ocp.constraints.uh_0 = (1e3 * np.ones((len(con_h),)))
            self.__ocp.constraints.lsh_0 = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush_0 = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh_0 = np.array(range(len(con_h)))    
            self.__ns_0+=len(con_h)

            
            
            #self.__ocp.constraints.lsh_0 = np.zeros(nsh)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            #self.__ocp.constraints.ush_0 = np.zeros(nsh)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            #self.__ocp.constraints.idxsh_0 = np.array(range(nsh))    # Jsh


            #self.__ocp.constraints.lsh_e = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            #self.__ocp.constraints.ush_e = np.zeros(len(con_h))             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            #self.__ocp.constraints.idxsh_e = np.array(range(len(con_h)))    # Jsh


            
        L2penalty=self.__L2_Norm
        L1pentaly=self.__L1_Norm
        
        
        ns = self.__ns_i
        self.__ocp.cost.zl = L1pentaly  * np.ones((ns,))    # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.Zl = L2penalty  * np.ones((ns,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.zu = L1pentaly  * np.ones((ns,))    
        self.__ocp.cost.Zu = L2penalty  * np.ones((ns,))  



        ns_e =self.__ns_e
        self.__ocp.cost.zl_e = L1pentaly * np.ones((ns_e,))    # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.Zl_e = L2penalty * np.ones((ns_e,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.zu_e = L1pentaly * np.ones((ns_e,))    
        self.__ocp.cost.Zu_e = L2penalty * np.ones((ns_e,)) 

        ns_0 =self.__ns_0
        self.__ocp.cost.zl_0 = L1pentaly * np.ones((ns_0,))    # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.Zl_0 = L2penalty * np.ones((ns_0,))    # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.__ocp.cost.zu_0 = L1pentaly * np.ones((ns_0,))    
        self.__ocp.cost.Zu_0 = L2penalty * np.ones((ns_0,)) 

    def __solver_compiler(self):
        x_ref = np.zeros(self.__nx)
        u_ref = np.zeros(self.__nu)
        # initial state
        self.__ocp.constraints.x0 = x_ref
        self.__ocp.cost.yref = np.concatenate((x_ref, u_ref))
        self.__ocp.cost.yref_e = x_ref
        #self.__ocp.solver_options.reg_epsilon='CONVEXIFY'
        self.__ocp.solver_options.qp_solver = self.__QP_solver
        self.__ocp.solver_options.nlp_solver_type = self.__nlp_solver_type
        self.__ocp.solver_options.tf = self.__Tf
        self.__ocp.solver_options.integrator_type=self.__integrator_type
        self.__ocp.solver_options.nlp_solver_max_iter=self.__nlp_solver_max_iter
        self.__ocp.solver_options.qp_solver_warm_start=1

        json_file = os.path.join('_acados_ocp.json')
    
        self.__solver = AcadosOcpSolver(self.__ocp, json_file=json_file)
        self.__integrator = AcadosSimSolver(self.__ocp, json_file=json_file)
        #if self.__cythonsolver==True:
        if False:
            AcadosOcpSolver.generate(self.__ocp, json_file='acados_ocp.json')
            AcadosOcpSolver.build(self.__ocp.code_export_directory, with_cython=True)
            self.__solver = AcadosOcpSolver.create_cython_solver('acados_ocp.json')

    def controller(self,x):

        x_current = x[0]
        self.__solver.set(0, 'lbx', x_current)
        self.__solver.set(0, 'ubx', x_current)
        if self.__print_status:
            self.__solver.print_statistics()

        status = self.__solver.solve()
        if status != 0 :
            print('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            pass
        self.solver_status=status

        u_list=[]
        x_list=[]
        for i in range(self.__N):

            u_list.append(self.__solver.get(i, 'u'))
            x_list.append(self.__solver.get(i,'x'))
        self.u_list=np.asarray(u_list)
        self.x_list=np.asarray(x_list)
        for i in range(len(self.x_list)):
            self.__solver.set(i, 'x', x_list[i])
            self.__solver.set(i, 'u', u_list[i])
            pass
        return self.u_list, self.x_list
    
    def set_controller_trajectory(self,traj,index):

        Q=self.__Q
        for i in range(self.__N):
            self.__solver.set(i, 'yref', np.concatenate((traj[i, :], np.zeros(2))))               
            self.__solver.cost_set(i, 'W', scipy.linalg.block_diag(Q, self.__R))
            Q = Q + (i / len(traj)) * (self.__Q_e - Q)
        self.__solver.cost_set(self.__N, 'W', Q)
        self.__solver.set(self.__N, 'yref', traj[-1,:])        
        Q_matrix_waypoint =  np.diag(self.__Q_matrix_waypoint) # [x,y,x_d,y_d,th,th_d]
        if index != None:
            if len(index)>0:
                if index[0]==self.__N:
                    self.__solver.cost_set(self.__N, 'W', Q_matrix_waypoint)
                else:
                    self.__solver.cost_set(index[0], 'W', scipy.linalg.block_diag(Q_matrix_waypoint, self.__R))

    def simulator(self,x,u):
        self.__integrator.set('x', x)
        self.__integrator.set('u', u)
        self.__integrator.solve()
        xcurrent=self.__integrator.get('x')
        return xcurrent.reshape(1,-1)
    


