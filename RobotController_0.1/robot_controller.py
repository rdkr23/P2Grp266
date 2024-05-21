import math as m


class RobotControllerSettings:
    """
    Robot controller settings class
    """
    def __init__(self):
        """
        Settings for the robot controller
        """
        self.update_frequency = 30 # hz
        self.linear_frequency = 10 # hz

        self.dh_values = {
            "alpha": [0, m.pi / 2, 0, 0, -m.pi / 2, m.pi / 2],
            "a": [0, 0, 135, 120, 0, 0],
            "d": [173.900, 0, 0, 88.78, 95, 65.5],
            "theta": [m.pi / 2, m.pi / 2, 0, -m.pi / 2, 0, 0]
        }


