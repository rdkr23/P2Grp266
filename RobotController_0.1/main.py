from robot_movement import MoveJ, MoveL, Robot, Kinematics, RobotMyCobot, RobotMyCobotAndSim
from gui import GUI
from robodk.robolink import *
from pymycobot.mycobot import MyCobot
from communication import CommunicationProtocol


import threading
import commands as cmd
import time as t
import math as m


BOTH = True
SIMULATING = False
Z_OFFSET = 0
STORAGE_OFFSET_X = 30
SYRINGE_OFFSET_X = -18
SYRINGE_MOVEMENT_Z = 25
SPEED = 1/0.5
ALLERGEN_AMOUNT = str(150)
END_EFFECTOR_TILT_TAKE = 85
END_EFFECTOR_TILT_PLACE = 89


PATCH_TEST_CORNER_COORDINATES = [-215, -265, 215]


SYRINGE_COORDINATES = [
    [305, 169.5+5, 153.5+5],
    [310, 94.5, 153.5+5],
    [310, 19.5, 153.5+5],
    [310, -65.5, 153.5+5],
    [305, -140.5, 153.5+5],
    [310, 132+5, 78],
    [310, 57, 76],
    [310, -28-5, 76],
    [310, -106-5, 76],
    [310, -184-5, 78]
]


performing_command = False


def communication_multi_thread(communicator, command):
    """
    Executes a command on the arduino using multi_threading
    :param communicator: instance of the communication class
    :param command: command to execute.
    """
    global performing_command

    performing_command = True
    communicator.send_command(command)
    performing_command = False


class Main:
    """
    Main class containing the whole program.
    """
    def __init__(self):
        """
        Connecting to ROBODK or MyCobot.
        """
        self.communication = CommunicationProtocol("COM3", 9600, 0.1)
        t.sleep(5)

        self.patch_test_config = [0, 1,
                                  2, 3,
                                  4, 5,
                                  6, 7,
                                  8, 9]

        self.robot = None

        self.gui_created = False
        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.start()

        while not self.gui_created:
            continue

        while not self.gui.running:
            continue

        self.set_mode(self.gui.mode)

    def create_gui(self):
        """
        Creates the GUI
        """
        self.gui = GUI()

        self.gui_created = True

        self.gui.run()

    def set_mode(self, mode):
        """
        Sets the mode for how the robot should run
        :param mode: either "both", "simulation", "robot".
        """
        if mode == "both":
            RDK = Robolink()
            rdk_link = RDK.Item('My Mechanism')
            my_cobot = MyCobot('COM4', 115200)

            self.robot = RobotMyCobotAndSim(my_cobot, rdk_link)
            self.robot.set_joints([0, 0, 0, 0, 0, 0], robot_speed=20)
            #self.communication.send_command(cmd.HOME_PRESSER)
            #self.communication.send_command(cmd.PREPARE_PRESSING)
            #self.communication.send_command(cmd.HOME_GRIPPER)
            #self.communication.send_command(cmd.CLOSE_GRIPPER)

        elif mode == "simulation":
            RDK = Robolink()
            rdk_link = RDK.Item('My Mechanism')
            self.robot = Robot(rdk_link)

        elif mode == "robot":
            my_cobot = MyCobot('COM4', 115200)

            my_cobot.set_free_mode(0)
            self.robot = RobotMyCobot(my_cobot)
            self.robot.set_joints([0, 0, 0, 0, 0, 0], robot_speed=20)

        self.run()

    def run(self):
        """
        Runs the whole program sequence
        """
        # Homing sekvens

        self.communication.send_command(cmd.HOME_PRESSER)
        self.communication.send_command(cmd.HOME_GRIPPER)

        self.communication.send_command(cmd.NEW_TEST)

        self.robot.set_joints([107.38158881112184, 46.157985403753784, 103.3563681945703, 30.485646401675925, 107.38158881112184, 135.0], robot_speed=20)

        t.sleep(5)

        for patch_text_index_number in range(len(self.patch_test_config)):

            self.take_allergen(self.patch_test_config[patch_text_index_number])

            MoveL(self.robot, [[260, -132-37.5, 153.5-Z_OFFSET, -90, 90, -90, 1.75 * SPEED]]).move_joint()
            MoveJ(self.robot, [[0, -350, PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET, -90, 90, 135+45, 0.5 * SPEED], [PATCH_TEST_CORNER_COORDINATES[0], PATCH_TEST_CORNER_COORDINATES[1], PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET, -90, 90, 135, 1 * SPEED]]).move_joint()

            self.dispense_allergen(patch_text_index_number)

            MoveJ(self.robot, [[0, -350, PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET, -90, 90, 135+45, 0.5 * SPEED], [260, -132-37.5, 153.5-Z_OFFSET, -90, 90, -90, 1 * SPEED]]).move_joint()

            self.place_allergen(self.patch_test_config[patch_text_index_number])

            if not self.gui.running:
                break

        self.gui.running = False
        self.gui.stop_pressed()

        # Waiting on a new start
        while not self.gui.running:
            continue

        self.set_mode(self.gui.mode)

    def dispense_allergen(self, index):
        """
        Runs the movements needed to dispense allergen onto a patch test
        :param index: What index the position on the patch test has
        :return: None
        """
        x_offset = (index % 2) * 20.5
        y_offset = m.floor(index / 2) * 20.5

        MoveL(self.robot, [[PATCH_TEST_CORNER_COORDINATES[0] - x_offset, PATCH_TEST_CORNER_COORDINATES[1] + y_offset, PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET, -90, 90, 135, 1 * SPEED]]).move_joint()
        MoveL(self.robot, [[PATCH_TEST_CORNER_COORDINATES[0] - x_offset, PATCH_TEST_CORNER_COORDINATES[1] + y_offset, PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET-10, -90, 90, 135, 0.5 * SPEED]]).move_joint()

        while performing_command:
            continue

        self.communication.send_command(cmd.PRESS_ALLERGEN + ALLERGEN_AMOUNT)

        MoveL(self.robot, [[PATCH_TEST_CORNER_COORDINATES[0] - x_offset, PATCH_TEST_CORNER_COORDINATES[1] + y_offset, PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET, -90, 90, 135, 0.5 * SPEED]]).move_joint()

        thread = threading.Thread(target=communication_multi_thread, args=(self.communication, cmd.HOME_PRESSER))
        thread.start()

        MoveL(self.robot, [[PATCH_TEST_CORNER_COORDINATES[0], PATCH_TEST_CORNER_COORDINATES[1], PATCH_TEST_CORNER_COORDINATES[2]-Z_OFFSET, -90, 90, 135, 1 * SPEED]]).move_joint()

    def take_allergen(self, index):
        """
        Takes an allergen from the storage
        :param index: Index for the allergen in the storage system
        :return: None
        """

        x = SYRINGE_COORDINATES[index][0]
        y = SYRINGE_COORDINATES[index][1]
        z = SYRINGE_COORDINATES[index][2]

        MoveL(self.robot, [[x + SYRINGE_OFFSET_X - STORAGE_OFFSET_X, y, z - Z_OFFSET, 0, END_EFFECTOR_TILT_TAKE, 0,
                            1.75 * SPEED]]).move_joint()

        MoveL(self.robot, [[x + SYRINGE_OFFSET_X, y, z - Z_OFFSET, 0, END_EFFECTOR_TILT_TAKE, 0,
                            0.75 * SPEED]]).move_joint()

        self.communication.send_command(cmd.CLOSE_GRIPPER)

        thread = threading.Thread(target=communication_multi_thread, args=(self.communication, cmd.PREPARE_PRESSING))
        thread.start()

        MoveL(self.robot, [
            [x + SYRINGE_OFFSET_X, y, z + SYRINGE_MOVEMENT_Z - Z_OFFSET, 0, END_EFFECTOR_TILT_TAKE, 0,
             0.75 * SPEED]]).move_joint()
        MoveL(self.robot, [
            [x + SYRINGE_OFFSET_X - STORAGE_OFFSET_X, y, z + SYRINGE_MOVEMENT_Z - Z_OFFSET, 0, END_EFFECTOR_TILT_TAKE, 0,
             0.75 * SPEED]]).move_joint()

    def place_allergen(self, index):
        """
        Places an allergen in storage
        :param index: Index for storage place
        :return: None
        """

        x = SYRINGE_COORDINATES[index][0]
        y = SYRINGE_COORDINATES[index][1]
        z = SYRINGE_COORDINATES[index][2]

        MoveL(self.robot, [[x + SYRINGE_OFFSET_X - STORAGE_OFFSET_X, y, z + SYRINGE_MOVEMENT_Z - Z_OFFSET, 0, END_EFFECTOR_TILT_PLACE, 0, 1.75 * SPEED]]).move_joint()
        MoveL(self.robot, [[x + SYRINGE_OFFSET_X, y, z + SYRINGE_MOVEMENT_Z - Z_OFFSET, 0, END_EFFECTOR_TILT_PLACE, 0, 0.75 * SPEED]]).move_joint()
        MoveL(self.robot, [[x + SYRINGE_OFFSET_X, y, z - Z_OFFSET, 0, END_EFFECTOR_TILT_PLACE, 0, 0.75 * SPEED]]).move_joint()

        while performing_command:
            continue

        self.communication.send_command(cmd.HOME_GRIPPER)

        MoveL(self.robot, [[x + SYRINGE_OFFSET_X - STORAGE_OFFSET_X, y, z - Z_OFFSET, 0, END_EFFECTOR_TILT_PLACE, 0, 0.75 * SPEED]]).move_joint()


if __name__ == "__main__":
    t.sleep(2)

    main = Main()

