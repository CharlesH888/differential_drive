import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from  matplotlib import animation as animation
from controller import Controller
from config import Config


class Plot():

    def __init__(self):
        self.current_x = Config.start_position.x
        self.current_y = Config.start_position.y
        self.current_theta = Config.start_position.theta

        self.x_path = []
        self.y_path =[]
        self.theta_path = []
        self.linear_velocity = []
        self.angular_velocity = []
        self.left_wheel_velocity = []
        self.right_wheel_velocity = []

        self.target_count = 0

    def plot_path(self, targets):

        fig= plt.figure()
   
        s1 = fig.add_subplot()
        
        plt.title("Differential Drive Sim", fontdict={'fontsize': 16, 'fontweight': 'bold'}, loc='center', pad=20)
        
        s1.set_facecolor('tab:green')

        x_data, y_data = [],[]
        line, = s1.plot([],[], 'g-', label='Path')
        line.set_linewidth(10)

        annotation_1 = s1.annotate("", xy=(0, 0), xytext=(-170, -100),textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="w"))
        annotation_2 = s1.annotate("", xy=(0, 0), xytext=(-170, -120),textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="w"))
                    
        s1.set_xlim(-12, 12)
        s1.set_ylim(-7,7)
        s1.grid(True)

        targets_reached =0
        for target in targets:
            #Initialize controller
            controller = Controller(target.x, target.y,self.current_x, self.current_y, self.current_theta)
            
            # Run PID controller to manuver the bot to each target
            x_, y_, theta_,linear_, angular_, left_, right_, target_reached = controller.runPID()
            if(target_reached): targets_reached += 1
            # append the path history from most recent target to a full path history that will be plotted
            self.x_path.extend(x_)
            self.y_path.extend(y_)
            self.theta_path.extend(theta_)
            self.linear_velocity.extend(linear_)
            self.angular_velocity.extend(angular_)
            self.left_wheel_velocity.extend(left_)
            self.right_wheel_velocity.extend(right_)


            # # Set the current position to the last value so it does not go back to the original start position
            self.current_x = self.x_path[-1]
            self.current_y = self.y_path[-1]
            self.current_theta = self.theta_path[-1]
            
            # Add targets to the plot
            s1.plot(target.x, target.y, 'ro', label='Goal')
        # get the total number of path points for animation length
        len_x = len(self.x_path)

        # Animation function for Path
        def update(frame):
            if frame == len_x-1:
                plt.close(fig)
            else:
                x_data.append(self.x_path[frame])
                y_data.append(self.y_path[frame])
                line.set_xdata(x_data)
                line.set_ydata(y_data)
                annotation_1.set_text(f"left wheel speed (rad/s) ={self.left_wheel_velocity[frame]:.2f}, right wheel speed (rad/s) ={self.right_wheel_velocity[frame]:.2f}")
                annotation_2.set_text(f"linear velocity (m/s) ={self.linear_velocity[frame]:.2f}, angular velocity (rad/s) ={self.angular_velocity[frame]:.2f}")
                return (line,)
    
        point, = s1.plot([], [], 'bo') 
        point.set_markersize(15)
        
        def update_point(i):
            if i == len_x-1:
                plt.close(fig)
            else:
                x = self.x_path[i]
                y = self.y_path[i]
                point.set_data([x], [y])
                if(abs(targets[self.target_count].x - x) + abs(targets[self.target_count].y - y) < 0.2):
                    s1.plot(targets[self.target_count].x, targets[self.target_count].y, 'bo')
                    self.target_count +=1

                return point,
    
        # Call the animation functions
        point_ani = animation.FuncAnimation(fig, update_point, frames=len_x, interval=10, blit=False, repeat = False)
        # text = animation.FuncAnimation(fig, animate,frames = len_x, interval=0, blit=False, repeat=False)
        ani = animation.FuncAnimation(fig, update, frames = len_x, interval=10, blit=False, repeat = False)
        
        plt.show()
        plt.close('all')
        return targets_reached

