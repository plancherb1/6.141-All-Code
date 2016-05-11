from random import random
from math import pi
from math import sqrt
from math import sin
from math import cos

def wrap_angle(angle_in):
    angle_out = angle_in % 2*pi
    if angle_out == 0:
        angle_out = 2*pi
    return angle_out


# wrapper function to calc out the result of applying u to x for time t
def calc_result_vertex(x,control,total_t):
    u = control[0]
    phi = cotrol[1]
    
    # pendulum dynamics function for control and point
    fullDynamics(t,x,u,phi)
        theta = x[2]
        L = 1 #TBD NEED TO UPDATE
        x_dot = cos(theta)*u
        y_dot = sin(theta)*u
        theta_dot = tan(phi)*u/L
        x_ddot = -1*y_dot*theta_dot
        y_ddot = x_dot*theta_dot
        theta_ddot = 0 #since we can directly control the angle and it happens so fast I think this is always 0
        dynamics = [x_dot,y_dot,theta_dot,x_ddot,y_ddot,theta_ddot]
    
    # simulate
    ''' MATLAB FUNCTION NEEDED TO SIMULATE A CONTROL FORWARD TO GET THE NEW POINT WE NEED TO
    CONVERT TO PYTHON we may need to use scipy.integrate.odeint'''
    xs = ode45(@fullDynamics,[0,total_t],x)
    final_x = xs[-1]
    
    # wrap the angle
    final_x[2] = wrap_angles(final_x[2])

    return final_x,u,phi

def extend(closest_vert,point):
    # discretize the controls into some resonable space of speed and angle
    possible_us = []
    possible_phis = []
    for i in range(41):
        possible_us.append(i*0.25-5) # guess of [-5,5] by .25
    for i in range(17):
        possible_phis.append(i*pi/8) # guess of (-pi/2,pi/2) by pi/8
    possible_phis = map(lambda x: x-pi/2, possible_phis)
    # helper functions
    total_t = .1
    def dot(a,b):
        return reduce(lambda x,y: x+y, map(lambda item: item[0]+item[1],zip(a,b)))
    def norm(a):
        return math.sqrt(reduce(labda x,y: x+y, map(lambda x: x**2,a)))
    
    # find the control that moves most in the direction of the point
    # the final implimentation uses the dot product to map the velocity vector
    # into the space that is directed between the two points to find the u
    # which will work most in the desired direction
    delta = map(lambda item: item[0]-item[1],zip(point,closest_vert))
    delta[2] = wrap_angle(delta[2])
    final_control = (possible_us[0],possible_phis[0])
    best_relative_direction = float("-inf")
    for possible_u in possible_us:
        for possible_phi in possible_phis:
            # Compute one step dynamics update
            # correct me if I'm wrong but i think the dynamics are:
            # x_dot = cos(theta)*u
            # y_dot = sin(theta)*u
            # theta_dot = tan(phi)/L *u
            # where u is the speed to the motor and phi is the steering angle
            # and L is the length of the car (I googled a bunch and could be wrong)
            ''' ---------------------------- '''
            theta = closest_vert[2]
            L = 1 #TBD NEED TO UPDATE
            x_dot = cos(theta)*possible_u
            y_dot = sin(theta)*possible_u
            theta_dot = tan(possible_phi)*possible_u/L
            x_ddot = -1*y_dot*theta_dot
            y_ddot = x_dot*theta_dot
            theta_ddot = 0 #since we can directly control the angle and it happens so fast I think this is always 0
            dir_after = [x_dot,y_dot,theta_dot,x_ddot,y_ddot,theta_ddot]
            ''' ---------------------------- '''
            direction_after = dir_after / norm(dir_after)
            # then dot them to find the relative direction
            relative_direction = dot(delta,direction_after)
            if relative_direction > best_relative_direction:
                best_relative_direction = relative_direction
                final_control = (possible_u,possible_phi)

    # then simulate forward to get the result
    return = calc_result_vertex(closest_vert,final_control,total_t)

def closestVertex(rrt_verts,point):
    '''
    These LQR Versions would be better but I don't know how to do them all in Python yet
    so I think we can skip for now and use the slower euclidian -- note if you do
    implement you need to pass K around
    from control import lqr #http://python-control.sourceforge.net/manual/synthesis.html
    import numpy as np
    from scipy import linalg
    # The LQR gains are constant for this whole function so compute first
    K,S = compute_lqr(point);
    def compute_lqr(x0):
        x0[2] = wrap_angle(x0[2])
        # calc the LQR
        A = [0,1;-g*cos(x0(1)),-b] #TBD
        B = [0;1] #TBD
        Q = eye(2) #TBD
        R = 0.1 #TBD
        K,S = lqr(A,B,Q,R)
    return K,S
    def compute_distance_(x0,x,S):
        X_bar = map(lambda item: item[0]-item[1],zip(x,x0));
        X_bar[2] = wrap_angle(X_bar[2]);
        #Do the matrix algebra to compute the difference
        X_bar = np.array(X_bar)
        X_bar_T = X_bar.T
        distance = X_bar_T.dot(S.dot(X_bar)); 
    return distance
    '''
    def compute_distance(point,vert):
        X_bar = map(lambda item:(item[0]-item[1])**2,zip(point,vert))
        distance = reduce(labda item,rest: item+rest, X_bar)
    # then loop around checking distance
    best_distance = float("inf")
    best_vert = rrt_verts[0]
    best_parent = 0
    for i in range(len(rrt_verts))
        vert = rrt_verts[i]
        distance = compute_distance(point,vert)
        if distance < best_distance
            best_distance = distance
            best_vert = vert
            best_parent = i
    return best_vert,best_parent

def pathFromStartToGoal(nodes,parents,controls,start)
    # initialize things
    xpath = []
    xpath[0] = nodes[-1];
    upath = []
    upath[0] = controls[-1];
    curr_index = len(nodes) - 1
    
    # then loop until you find the start
    done = 0;
    while ~done
        # get the parent
        next_node = nodes[parents[curr_index]]
        next_control = controls[parents[curr_index]]
        # add to path
        xpath.append(next_node)
        upath.append(next_control)
        # check if done
        if next_node == start
            break
        # else get the next index
        curr_index = parents[curr_index];

    # then reverse the path
    xpath = xpath.reverse()
    upath = upath.reverse()
    return xpath,upath

def runRRT(start,goal):
    # Bounds on world
    world_bounds_x = [0,1] #UPDATE ME
    world_bounds_y = [0,1] #UPDATE ME
    world_bounds_theta = [0,2*pi] #UPDATE ME
    world_bounds_x_dot = [-5,5] #UPDATE ME
    world_bounds_y_dot = [-5,5] #UPDATE ME
    world_bounds_theta_dot = [-5,5] #UPDATE ME

    #TBD Data structure to hold the RRT nodes and parents to recreate the traj
    rrt_verts = []
    rrt_parents = []
    rrt_controls = []

    # tune this to get it to finish reasonably
    minDistGoal = 1; #UPDATE ME

    # run RRT until we reach the goal
    while ~nearGoal
        # get a random sample point
        rnd = random()
        # With probability 0.05, sample the goal. This promotes movement to the goal
        if rnd < 0.05
            point = goal;
        else
            # Sample from space with probability 0.95 to get random point
            x = (world_bounds_x[1] - world_bounds_x[0])*random() + world_bounds_x[0]
            y = (world_bounds_y[1] - world_bounds_y[0])*random() + world_bounds_y[0]
            theta = (world_bounds_theta[1] - world_bounds_theta[0])*random() + world_bounds_theta[0]
            x = (world_bounds_x_dot[1] - world_bounds_x_dot[0])*random() + world_bounds_x_dot[0]
            y = (world_bounds_y_dot[1] - world_bounds_y_dot[0])*random() + world_bounds_y_dot[0]
            theta = (world_bounds_theta_dot[1] - world_bounds_theta_dot[0])*random() + world_bounds_theta_dot[0]
            point = [x,y,theta,x_dot,y_dot,theta_dot]
        # Then find the closest vertex in the rrt to this point
        closest_vert,parent_index = closestVertex(rrt_verts,point)
        # Then extend in that direction
        new_vert,control = extend(closest_vert,xy)
        # And add to the data structure
        rrt_verts.append(new_vert)
        rrt_parents.append(parent_index)
        rrt_controls.append(control)
        # If close to goal end
        if abs(goal-new_vert) < minDistGoal:
            break

    # then compute the finished path
    path,controls = pathFromStartToGoal(rrt_verts,rrt_parents,rrt_controls,start)
    
        
