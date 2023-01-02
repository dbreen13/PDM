import numpy as np
import cvxpy as cp

#the sampling time
dt=0.01

class dynamics:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x #the x-position
        self.y = y #the y-position
        self.yaw = yaw #the yaw
        self.v = v #the velocity
        self.L=0.2 #length of the wheel base
        
    def next_state(self,X,u):
        #unpack state and input
        x,y,phi,v=X
        delta,a=u
        
        #update the state
        x_dot=v*np.cos(phi)
        y_dot=-v*np.sin(phi)
        v_dot=a
        phi_dot=(v*np.tan(delta))/self.L
        next_state=np.array([x+x_dot*dt,y+y_dot*dt,v+v_dot*dt,phi+phi_dot*dt])
        
        return next_state

    def linearized_model(self,v,phi,delta):
        #the linearized  and discretized state space
        A=np.array([[1,0,np.cos(phi)*dt, -v*np.sin(phi)*dt],[0,1,np.sin(phi)*dt, v*np.cos(phi)*dt],[0,0,1,0],[0,0,np.tan(delta)/self.L*dt]],dtype=object)
        B=np.array([[0,0],[0,0],[dt,0],[0, v/(self.L*np.cos(delta))*dt]],dtype=object)
        C=np.array([v*np.sin(phi)*phi*dt, -v*np.cos(phi)*phi*dt, 0, (-v*delta)/(self.L*(np.cos(delta))**2)*dt])
        return A,B,C

    def prediction_model(x_init,delta_mpc,a_mpc,x_ref,T):
        x_pred=x_ref*0.0
        for i in range(len(x_init)):
            x_pred[i,0]=x_init[i]
        state=dynamics(x=x_init[0], y=x_init[1], yaw=x_init[2], v=x_init[3])
        for (a_pred,d_pred,pred) in zip(a_mpc,delta_mpc, range(1,T+1)):
            u=np.array([d_pred,a_pred])
            state_new=dynamics.next_state(state,u)
            x_pred[0,i]=state_new[0]
            x_pred[1,i]=state_new[1]
            x_pred[2,i]=state_new[2]
            x_pred[3,i]=state_new[3]
        return x_pred

class controller:
    def __init__(self):
        #the size of input and state
        self.nx=4 #number of states
        self.nu=2 #number of inputs
        
        #the mpc controller variables
        self.pred_hor=20
        self.control_hor=2
        
        #the input and control weights
        self.Q=10*np.eye(self.nx) #weight on the states
        self.R1=0.1*np.eye(self.nu) #weight on the inputs
        self.R2=0.1*np.eye(self.nu) #weight on the input difference
        
        #The constraints
        self.v_max=1
        self.v_min=-1
        self.delta_max=np.radians(30)
        self.a_max=1
        
        #the sampling time
        self.dt=0.01
        
    def mpc_controller(self,x_ref, x_pred, x_init, delta_ref):
        #create cvx variables
        x=cp.Variable((self.nx,self.pred_hor+1))
        u=cp.Variable((self.nu,self.pred_hor))
        
        #initialize the cost and constraints
        cost=0.0
        constraints=[]
        
        #start the MPC loop
        for t in range(self.pred_hor): #predicting the model over the prediction horizon
            cost += cp.quad_form(u[:, t],self.R1)
            
            if t != 0:
                cost += cp.quad_form(x_ref[:, t] - x[:, t],self.Q)
            
            A,B,C=dynamics.linearized_model(self,x_pred[2,t],x_pred[3,t],delta_ref)
            constraints+= [x[:, 0] == x_init]
            constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]
            
            if t < (self.pred_hor - 1):
                cost += cp.quad_form(u[:, t + 1] - u[:, t], self.R2)
                constraints += [cp.abs(u[1, t + 1] - u[1, t]) <= self.delta_max * self.dt] #maximum steerspeed
            
        #constant state constraints
        constraints += [x[2,:]<=self.v_max]
        constraints += [x[2,:]>=self.v_min]
        constraints += [cp.abs(u[1,:])<=self.delta_max]
        constraints += [cp.abs(u[0,:])<=self.a_max]
        
        #solve the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP)
        
        if problem.status != cp.OPTIMAL or problem.status != cp.OPTIMAL_INACCURATE:
            print('MPC not solvable')
            
        #get the states from the solver
        x_mpc= np.array(x.value[0,:]).flatten()
        y_mpc= np.array(x.value[1,:]).flatten()
        v_mpc=np.array(x.value[2,:]).flatten()
        phi_mpc=np.array(x.value[3,:]).flatten()
        a_mpc=np.array(u.value[0,:]).flatten()
        delta_mpc=np.array(u.value[1,:]).flatten()
        
        
        return x_mpc,y_mpc,v_mpc,phi_mpc,a_mpc,delta_mpc
        
        
    

