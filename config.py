from state import State

class Config():
    wheel_radius = .3  #  Meters
    wheel_base = 1  # Meters

    max_speed = 1 # m/s
    max_angular_speed = 1.5 # rad/s
    min_angular_speed = -1.5 # rad/s

    angular_speed_step = 0.1 # rad/s
    linear_speed_step = 0.05 # m/s

    P = 1.0
    I = 0.001
    D = 0.001

    dt = 0.1  # Seconds
    arrive_clearance = 0.1 # Meters

    start_position = State(0, 0, 0)
    target_locations = [State(5, 5, 0),
        State(-10, 3, 0),
        State(10, -3, 0),
        State(-2, -2, 0)]