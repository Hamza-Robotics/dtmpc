
def controller(self,x,traj):
    x_current = x[0]
    x=x_current[0] 
    y=x_current[1] 
    th=x_current[2]
    e_d=np.sqrt(e_x**2+e_y**2)
    e_o=(e_y*ca.cos(th))/e_d-(e_x*ca.sin(th))/e_d
    state = np.array([x, y,th,
                      x,y, 
                      0.0, 
                      0,0])
    self.__solver.set(0, 'lbx', state)
    self.__solver.set(0, 'ubx', state)
    
    
    
    Q=self.__Q
    for i in range(self.__N):
        xd=traj[i,0]
        yd=traj[i,1]
        e_x=x-xd
        e_y=y-yd

        e_d=np.sqrt(e_x**2+e_y**2)
        e_o=(e_y*np.cos(th))/e_d-(e_x*np.sin(th))/e_d
        current_traj=np.array([x,y, th,
                               xd, yd, 
                               0.4, 0.4, 
                               e_d, e_o])
        self.__solver.set(i, 'yref', np.concatenate((current_traj, np.zeros(2))))               
        self.__solver.cost_set(i, 'W', scipy.linalg.block_diag(Q, self.__R))
        Q = Q + (i / len(traj)) * (self.__Q_e - Q)


    self.__solver.cost_set(self.__N, 'W', Q)
    xd=traj[-1,0]
    yd=traj[-1,1]
    e_x=x-xd
    e_y=y-yd
    e_d=np.sqrt(e_x**2+e_y**2)
    e_o=(e_y*np.cos(th))/e_d-(e_x*np.sin(th))/e_d
    current_traj=np.array([x,y, th,
                    xd, yd, 
                    0.4, 0.4, 
                    e_d, e_o])
    self.__solver.set(self.__N, 'yref',current_traj)        
    if self.__print_status:
        self.__solver.print_statistics()

    status = self.__solver.solve()
    if status != 0 :
        print('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
        pass
    self.solver_status=status        
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