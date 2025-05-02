## Install build dependecies (Assuming Python already is):

numpy: 

    - pip install numpy

matplotlib: 

    - pip install matplotlib

unittest: 

    - pip install matplotlib

## Differential Drive PID Controller Simulation

- In this project, I have modeled differential drive kinematics and implemented waypoint navigations with a PID controller

- The systems take in a list of waypoints in the form of arrays
    -   Each waypoint target has an X coordinate, Y coordiante and a Theta
    - When the bot gets within a configuable distance to the target waypoint, the waypoint is marked as reached and the next is given to the controller

- This project creates an animation of the bot navigating and reports out wheel velocities, linear velocity and angular velocity
- When the bot reaches a waypoint, the animation confirms it




https://github.com/user-attachments/assets/ee30bd75-0b1e-476b-bfdf-3201c64b6a31

## Follow these steps to run this project:

    git clone https://github.com/CharlesH888/differential_drive.git

    cd differential_drive

    python main.py

## What are the controller’s limitations?

- Controller has no situational awareness. If there was an obstacle in the path, it could not deviate

- PID controllers are easy to implement but and work great in simulation when there is no noise, but can be less stable in physical deployments.

    
## How could you improve performance?

- Due to the fact that I am adjusting linear speed based on linear error, the bot is immediately increasing linear speed when a new waypoint is given. Differential Drive systems have the ability to spin in place (closing angular error but not linear), but this requires seperating linear and angular error reduction. Seperating could be more efficient as more path is traveled making larger sweeping turns that just spinning in place. 

- I made the decision not to seperate because the application this is designed for is an outdoor autonomous robot. Spinning in place is not always ideal with uncertain terrain and can cause bots to get stuck. Keeping forward linear motion also simplifies the requirements for situational awareness. 

- Controller also allows for instantaneous changes to desired speed. This is acceptable for simulation, but real systems cannot and should not attempt to make dramatic changes in speed. Another controller for speed should be implemented for a physical implementation. 

- Currently the PID controller has not been tuned and would require more effort to optimize performance

## Screenshots:

![Figure_4](https://github.com/user-attachments/assets/cb92a66d-82af-48d3-8393-03ac22923855)
![Figure_1](https://github.com/user-attachments/assets/07e49083-31cc-4a20-acc7-0a33b113d172) 
![Figure_5](https://github.com/user-attachments/assets/51ef07fd-dd49-452b-9ce7-0021c89fe58b)
