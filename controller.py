import numpy as np
from config import Config

class Controller():
    def __init__(self, target_x, target_y, current_x, current_y, current_theta):

        self.current_x = current_x
        self.current_y = current_y
        self.current_theta = current_theta
        self.target_x = target_x
        self.target_y = target_y
        self.wheel_radius = Config.wheel_radius 
        self.wheel_base = Config.wheel_base 


        self.max_speed = Config.max_speed
        self.max_angular_speed = Config.max_angular_speed
        self.min_angular_speed = Config.min_angular_speed

        self.P = Config.P
        self.I = Config.I
        self.D = Config.D

        self.dt = Config.dt 
        self.arrival_clearance = Config.arrive_clearance

        return

    def calculate_wheel_velocities(self, v, w):
        """ Returns rotational velocity of the wheels
        """
        vR = (2*v + w*self.wheel_base)/(2*self.wheel_radius)
        vL = (2*v - w*self.wheel_base)/(2*self.wheel_radius)
        return vR, vL

    
    def normalize_angle(self, angle):
        """ Returns the angle normalized between pi through -pi
        """
        return np.arctan2(np.sin(angle), np.cos(angle))

    def update_xy(self, v, w):
        """ Updates current X, Y and Theta values of the bot
        """
        x_dt = v*np.cos(self.current_theta)
        y_dt = v*np.sin(self.current_theta)
        theta_dt = w

        self.current_x = self.current_x + x_dt * self.dt
        self.current_y = self.current_y + y_dt * self.dt
        self.current_theta = self.normalize_angle(self.current_theta + self.normalize_angle(theta_dt * self.dt))
        
        return
    
    def pid_controller(self, e, e_prev, e_acc, delta_t, kp, ki, kd):
        """ PID controller return the adjusted values based on the controller, 
            as well as the updated previous value and accumulated error
            PID algortithm: is executed every delta_t seconds
            The error e is calculated as: e = desired_value - actual_value
            e_prev contains the error calculated in the previous step.
            e_acc contains the integration (accumulation) term.
        """

        P = kp*e                        # Proportional term; kp is the proportional gain
        I = e_acc + ki*e*delta_t        # Intergral term; ki is the integral gain
        D = kd*(e - e_prev)/delta_t     # Derivative term; kd is the derivative gain

        output = P + I + D              # controller output

        # store values for the next iteration
        e_prev = e     # error value in the previous interation (to calculate the derivative term)
        e_acc = I      # accumulated error value (to calculate the integral term)

        return output, e_prev, e_acc

    def arrive_check(self):
        """ Returns True or False if the bot is within 
            the acceptable distance to the target point
        """
        
        current_xy = np.array([self.current_x, self.current_y])
        target_xy = np.array([self.target_x, self.target_y])
        difference = abs(current_xy - target_xy)

        distance_error = difference[0] + difference [1]
        if distance_error < self.arrival_clearance:
            return True
        else:
            return False


    def get_pose_error(self, tar_x, tar_y, x, y, theta):
        """ Returns the position and orientation errors. 
            Orientation error is bounded between -pi and +pi radians.
        """
        # Position error:
        x_err = tar_x - x
        y_err = tar_y - y
        dist_err = np.sqrt(x_err**2 + y_err**2)

        # Orientation error
        rot_d = np.arctan2(y_err,x_err)
        rot_err = rot_d - theta

        # Limits the error to (-pi, pi):
        rot_err_correct = np.arctan2(np.sin(rot_err),np.cos(rot_err))

        return dist_err, rot_err_correct
    
 
    def runPID(self):
        """ Returns X nad Y path history from the last waypoint, 
            as well as the history of the bot's heading, wheel velocities
            and linear and rotational velocities
        """
        # Get and initial Error for the starting location
        dist_err, rot_err = self.get_pose_error(self.target_x, self.target_y, self.current_x,self.current_y,self.current_theta)
        
        #Initialize the path history with the starting locations
        x = [self.current_x]
        y = [self.current_y]
        theta = [self.current_theta]
        linear_vel = [0]
        angular_vel = [0]
        left_wheel_vel = [0]
        right_wheel_vel = [0]
        
        #initialize error values so they are not null
        e_previous_rot = 0.9*rot_err
        e_accumulated_rot = 0
        e_previous_linear = 0.9*rot_err
        e_accumulated_linear = 0
        
        #
        while(not self.arrive_check()):
            dist_err, rot_err = self.get_pose_error(self.target_x, self.target_y, self.current_x,self.current_y,self.current_theta)
            desired_angular, e_previous_rot, e_accumulated_rot = self.pid_controller(rot_err, e_previous_rot, e_accumulated_rot, self.dt, self.P, self.I, self.D)
            desired_linear, e_previous_linear, e_accumulated_linear = self.pid_controller(dist_err, e_previous_linear, e_accumulated_linear, self.dt, self.P, self.I, self.D)
            if(desired_linear > self.max_speed):
                desired_linear = self.max_speed
            if(desired_angular > self.max_angular_speed):
                desired_angular = self.max_angular_speed
            if(desired_angular < self.min_angular_speed):
                desired_angular = self.min_angular_speed
            Vr, Vl = Controller.calculate_wheel_velocities(self, desired_linear, desired_angular)
            
            # Updates the current x y and theta of the bot
            self.update_xy(desired_linear, desired_angular )

            # stores the new position in a path history buffer
            x.append(self.current_x)
            y.append(self.current_y)
            theta.append(self.current_theta)
            linear_vel.append(desired_linear)
            angular_vel.append(desired_angular)
            left_wheel_vel.append(Vl)
            right_wheel_vel.append(Vr)


           
        return x, y, theta, linear_vel, angular_vel, left_wheel_vel, right_wheel_vel, self.arrive_check()
    
    