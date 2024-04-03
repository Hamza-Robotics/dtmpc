#!/usr/bin/env python3
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel
import casadi as ca
from mobile_robot_model import MobileRobotModel
import yaml
import numpy as np
import scipy
import os

class  NMPC():

    def __init__(self):
        with open('src/mpc/config/dnmpc_params.yaml') as file:
            yamlfile = yaml.safe_load(file)
        self.__QP_solver = yamlfile['QP_solver']
        self.frequency=yamlfile['Prediction_Frequency']
        self.__prediction_length = yamlfile['Prediction_Length']
        self.update_frequency=yamlfile['Update_Frequency']
        self.__robot_model=self.__define_model()
        self.__initialized=False

        self.__Tf = self.__prediction_length
        self.__N=int(self.__prediction_length*self.frequency)
        self.N=self.__N 
        self.numberofobs=yamlfile['Number_obstacles']
        self.cost_map=0.0
        self.__model=self.__robot_model # Model
        self.__ocp = AcadosOcp()
        self.__ocp.model = self.__model #Define Model

        # Initialize Parameters.
        self.__n_params = self.__model.p.size()[0]
        self.max_v=yamlfile['velocity_max']
        self.min_v=yamlfile['velocity_min']
        self.max_th_d=np.deg2rad(yamlfile['angle_velocity_max'])
        self.min_th_d=np.deg2rad(yamlfile['angle_velocity_min'])
        self.prediction_length=self.__prediction_length
        self.__ocp.dims.np = self.__n_params
        self.__ocp.parameter_values = np.zeros(self.__n_params)
        self.__ocp.dims.N = self.__N

        # Vehicle parameters
        self.__nlp_solver_type=yamlfile['nlp_solver_type']
        self.__Q_matrix=yamlfile['Q_matrix'] 
        self.__Q_matrix_e=yamlfile['Q_matrix_e']       
        self.__R_matrix=yamlfile['R_matrix']
        self.__print_status=yamlfile['print_acados_status']
        self.__nlp_solver_max_iter=yamlfile['nlp_solver_max_iter']
        self.__integrator_type=yamlfile['integrator_type']
        self.__L2_Norm=yamlfile['L2_Norm']
        self.__L1_Norm=yamlfile['L1_Norm']
        self.__ed_max=yamlfile['ed_max']
        self.__eo_max=yamlfile['eo_max']
        self.__eo_min=yamlfile['eo_min']
        self.__ed_min=yamlfile['ed_min']



        self.__obstacle=yamlfile['obstacle']
        self.__numberofobs=yamlfile['Number_obstacles']


        self.__linear_cost_function()
        self.__constraints()
        self.__solver_compiler()
    

    def __generate_obstacle_params(self):
        obs=[]
        for i in range(1):
            obs_temp=np.array([ca.SX.sym('x'+str(i)+'_obs'),ca.SX.sym('y'+str(i)+'_obs'),ca.SX.sym('r'+str(i)+'_obs')]).reshape(-1,1)            
            obs.append(obs_temp)
        print("hehehehe",obs)
        return ca.vertcat(*obs)
    
    def __define_model(self):
        model=AcadosModel()
        # control inputs
        v = ca.SX.sym('vs')
        th_d = ca.SX.sym('th_d')
        controls = ca.vertcat(v, th_d)
        # n_controls = controls.size()[0]
        # model stat
    
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
        e_x=(x-x_d)
        e_y=(y-y_d)

        e_d = ca.SX.sym('e_d')
        e_o = ca.SX.sym('e_o')

        states = ca.vertcat(
                            x, 
                            y, 
                            th,
                            e_d,
                            e_o)
        obs=self.__generate_obstacle_params()
        paremeters  = ca.vertcat(x_d,y_d,xdot_d,ydot_d,obs)   
        print("pp",paremeters)


        eps=0.000001
        m11 = (e_x*ca.cos(th)+e_y*ca.sin(th))/(e_d+eps)
        m12 = 0
        m21 = -((e_y*ca.cos(th)-e_x*ca.sin(th))*(e_x*ca.cos(th)+e_y*ca.sin(th)))/(e_d**2+eps)
        m22 = -((e_x/(e_d+eps))*ca.cos(th)+(e_y/(e_d**2+eps))*ca.sin(th))
        
        
        dt=1/self.frequency
        e1 = -((e_x*xdot_d*dt+e_y*ydot_d*dt)/(e_d+eps))
        e2 = (e_o*(xdot_d*dt*e_x+ydot_d*dt*e_y)+e_d*(ca.sin(th)-ca.cos(th)))/(e_d**2+eps)
        
        J = ca.vertcat(ca.horzcat(m11,m12),
                       ca.horzcat(m21,m22))
        E=ca.vertcat(e1, e2)
        k=1
        kin_eq = [ca.vertcat(v*ca.cos(th)*k, 
                           (v)*ca.sin(th)*k,
                             th_d*k,
            (ca.mtimes(J, controls)+E))]   
 

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
        return model
    
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

        self.__ocp.cost.yref = np.ones((self.__ny,))
        self.__ocp.cost.yref_e = np.ones((self.__ny_e,))
        self.__ocp.constraints.x0 = np.ones((self.__nx,))
    
    def __constraints(self):
    

        self.__ns_e=0
        self.__ns_0=0
        self.__ns_i=0
        x_alg=self.__model.x[0]
        y_alg=self.__model.x[1]
        th_alg=self.__model.x[2]
        e_d_alg=self.__model.x[3]
        e_o_alg=self.__model.x[4]
        
        x_d_alg=self.__model.p[0]
        y_d_alg=self.__model.p[1]


        self.__ocp.constraints.lbu = np.array([self.min_v])
        self.__ocp.constraints.ubu = np.array([self.max_v])
        self.__ocp.constraints.idxbu = np.array([0])
        self.__ocp.constraints.lsbu = np.zeros(1)
        self.__ocp.constraints.usbu = np.zeros(1)
        self.__ocp.constraints.idxsbu = np.array([0])
        self.__ns_i+=1
        self.__ns_0+=1


        ex=x_alg-x_d_alg
        ey=y_alg-y_d_alg

  
        if True:
            con_h=[#(x_alg-(-.34))**2 + (y_alg-(+1.76))**2 - (0.1)**2,
                #e_d_alg**2,
                e_d_alg**2,

                #ca.sqrt(e_d_alg**2+e_o_alg**2+0.1),
                
                    ]
            con_h_vcat=ca.vcat(con_h)
            self.__ocp.model.con_h_expr =con_h_vcat
            self.__ocp.constraints.lh =np.array([self.__ed_min**2,
                                                ])
            self.__ocp.constraints.uh =np.array([self.__ed_max**2,
                                                ])
            self.__ocp.constraints.lsh = np.zeros(1)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush = np.zeros(1)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh = np.array([0])    # Jsh
            self.__ns_i+=1



            self.__ocp.model.con_h_expr_e =con_h_vcat
            self.__ocp.constraints.lh_e =np.array([self.__ed_min**2,
                                                    ])
            self.__ocp.constraints.uh_e =np.array([self.__ed_max**2,
                                                    ])
            self.__ocp.constraints.lsh_e = np.zeros(1)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush_e = np.zeros(1)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh_e = np.array([0])    # Jsh
            self.__ns_e+=1

  
        if False:



            ex=x_alg-x_d_alg
            ey=y_alg-y_d_alg
                
                    
            con_h=[#(x_alg-(-.34))**2 + (y_alg-(+1.76))**2 - (0.1)**2,
                #e_d_alg**2,
                ex**2+ey**2,
                #e_d_alg**2,
                
                
                    ]
            
            con_h_vcat=ca.vcat(con_h)
            self.__ocp.model.con_h_expr =con_h_vcat
            self.__ocp.constraints.lh =np.array([self.__ed_min**2,
                                                ])
            self.__ocp.constraints.uh =np.array([self.__ed_max**2,
                                                ])
            self.__ocp.constraints.lsh = np.zeros(1)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush = np.zeros(1)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh = np.array([0])    # Jsh
            self.__ns_i+=1



            self.__ocp.model.con_h_expr_e =con_h_vcat
            self.__ocp.constraints.lh_e =np.array([self.__ed_min**2,
                                                    ])
            self.__ocp.constraints.uh_e =np.array([self.__ed_max**2,
                                                    ])
            self.__ocp.constraints.lsh_e = np.zeros(1)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
            self.__ocp.constraints.ush_e = np.zeros(1)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
            self.__ocp.constraints.idxsh_e = np.array([0])    # Jsh
            self.__ns_e+=1



        if False:

            self.__ocp.constraints.lbx = np.array([0.1])
            self.__ocp.constraints.ubx = np.array([1])
            self.__ocp.constraints.idxbx = np.array([3])
            self.__ocp.constraints.lsbx = np.zeros(1)
            self.__ocp.constraints.usbx = np.zeros(1)
            self.__ocp.constraints.idxsbx = np.array([0])
            self.__ns_i+=1

            

            self.__ocp.constraints.lbx_e = np.array([0.1])
            self.__ocp.constraints.ubx_e = np.array([1])
            self.__ocp.constraints.idxbx_e = np.array([3])
            self.__ocp.constraints.lsbx_e = np.zeros(1)
            self.__ocp.constraints.usbx_e = np.zeros(1)
            self.__ocp.constraints.idxsbx_e = np.array([0])
            self.__ns_e+=1

                



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
        self.__ocp.solver_options.levenberg_marquardt = 1.0
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
    
    def __e_de_o(self,x,xd,y,yd,th):
        e_x=x-xd
        e_y=y-yd
        e_d=np.sqrt(e_x**2+e_y**2)
        e_o=(e_y*np.cos(th))/e_d-(e_x*np.sin(th))/e_d
        return e_d,e_o   
    
    def controller(self,x,traj,vel,obs,robots=None):
        
        
        x_current = x[0]
        x=x_current[0] 
        y=x_current[1] 
        th=x_current[2]
        e_d,e_o=self.__e_de_o(x,traj[0,0],y,traj[0,1],th)
        state = np.array([x,y,th,e_d,e_o])
        print(state)
        if not self.__initialized:
            self.__initialize(state)

        self.__set_controller_trajectory(x,y,th,traj,vel,obs)

        self.__solver.set(0, 'lbx', state)
        self.__solver.set(0, 'ubx', state)
        
        status = self.__solver.solve()
        if status != 0 :
            print('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            #raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
        
        if self.__print_status:
            self.__solver.print_statistics()
        self.solver_status=status

        u_list=[]
        x_list=[]
        z_list=[]
        pi_list=[]
        lam_list=[]
        t_list=[]
        sl_list=[]
        su_list=[]



        for i in range(self.__N):

            u_list.append(self.__solver.get(i, 'u'))
            x_list.append(self.__solver.get(i,'x'))
 
            
        
        self.u_list=np.asarray(u_list)
        self.x_list=np.asarray(x_list)
        self.z_list=np.asarray(z_list)
        self.pi_list=np.asarray(pi_list)

        
        
        for i in range(len(self.x_list)):
            self.__solver.set(i, 'x', x_list[i])
            self.__solver.set(i, 'u', u_list[i])

            pass
        self.__initialized=False 
        return self.u_list, self.x_list
    
    def __initialize(self,state):
        for i in range(self.__N):
            self.__solver.set(i, 'x', state)
            self.__solver.set(i, 'u', np.zeros((self.__nu,)))
            pass
        self.__solver.set(self.__N, 'x', state)
        #self.__initialized=True 

    def __set_controller_trajectory(self,x,y,th,traj,vel,obs,robots=None):

        Q=self.__Q
        Q_e=self.__Q_e
        for i in range(self.__N):
                
                self.__solver.set(i, 'yref', np.concatenate((np.zeros(3),np.array([0.1,0.0]),np.zeros(2)))) 
                print(self.__set_params(traj[i],vel[i],obs,1)[0])       
                self.__solver.set(i, 'p', self.__set_params(traj[i],vel[i],obs,robots))
                self.__solver.cost_set(i, 'W', scipy.linalg.block_diag(Q_e, self.__R))
                Q = Q - (i / len(traj)) * (Q-self.__Q_e)
          

        self.__solver.cost_set(self.__N, 'W', Q)
        self.__solver.set(self.__N, 'p', self.__set_params(traj[self.__N-1],vel[self.__N-1],obs,robots))
        self.__solver.set(self.__N, 'yref', np.concatenate((np.zeros(3),np.array([0.1,0.0]))))      
        

        if False:
            if index != None:
                if len(index)>0:
                    if index[0]==self.__N:
                        self.__solver.cost_set(self.__N, 'W', Q_matrix_waypoint)
                    else:
                        self.__solver.cost_set(index[0], 'W', scipy.linalg.block_diag(Q_matrix_waypoint, self.__R))

    def __set_params(self,traj,vel,obs,robots=None):
        xd=traj[0]
        yd=traj[1]
        xdotd=vel[0]
        ydotd=vel[1]
        
   
        params=np.array([xd,yd,xdotd,ydotd])
        for i in range(len(obs)):
            params = np.concatenate((params, obs[i]))
        return params
    
    def simulator(self,x,u):
        self.__integrator.set('x', x)
        self.__integrator.set('u', u)
        self.__integrator.solve()
        xcurrent=self.__integrator.get('x')
        return xcurrent.reshape(1,-1)
    


