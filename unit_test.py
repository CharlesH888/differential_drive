import unittest
from state import State
from plot import Plot
from controller import Controller
from config import Config

class TestPlot(unittest.TestCase):
    def test_case_plot(self):
        plot = Plot()
        self.assertEqual(plot.plot_path(
        [State(5, 5, 0),
        State(-10, 3, 0),
        State(10, -3, 0),
        State(-2, -2, 0)]), 4)

    def test_case_PID(self):
            controller = Controller(0,0,0,0,0)
            self.assertEqual(controller.runPID(), ([0],[0],[0],[0],[0],[0],[0],True))

    def test_case_pose_error(self):
            self.assertEqual(Controller.get_pose_error(self,0,0,0,0,0), (0,0))
    
    def test_case_arrive_check(self):
            controller = Controller(0,0,0,0,0)
            self.assertEqual(controller.arrive_check(), True)

    def test_case_PID_controls(self):
            controller = Controller(0,0,0,0,0)
            self.assertEqual(controller.pid_controller(1,0.9,0,0.1,1,0.1,0), (1.01,1,0.010000000000000002))   
   
    def test_case_update_xy(self):
            controller = Controller(0,0,0,0,0)
            self.assertEqual(controller.update_xy(0,0), None)

    def test_case_normalize_angle(self):
            self.assertEqual(Controller.normalize_angle(self,1), 1.0)
    
    def test_case_wheel_velocities(self):
            config = Config()
            controller = Controller(0,0,0,0,0)
            self.assertEqual(controller.calculate_wheel_velocities(0,0), (0,0))
if __name__ == "__main__":
    unittest.main()