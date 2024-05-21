import math as m
import time as t
import numpy as np
import random as r

from robot_controller import RobotControllerSettings
from numpy.linalg import inv

MIN_ANGLE_LINK_1_5 = -165 / 180 * m.pi
MAX_ANGLE_LINK_1_5 = 165 / 180 * m.pi
MIN_ANGLE_LINK_6 = -175 / 180 * m.pi
MAX_ANGLE_LINK_6 = 175 / 180 * m.pi

MOVE_NOT_POSSIBLE_ERROR_MSG = "MoveNotPossible: The given movement command can not be executed by the program. Please make sure that it dies not result in a singularity"
JOINT_SYNTAX_ERROR_MSG = "JointSyntaxError: The syntax must be [Joint1, joint2, joint3, joint4, joint5, joint6] for a joint"
TAKES_ONLY_TWO_COORDINATES_ERROR_MSG = "TakesOnlyTwoCoordinates: This class takes only two coordinates."
COORDINATE_SYNTAX_ERROR_MSG = "CoordinateSyntaxError: The syntax must be: [x, y, z, alpha, beta, gamma, time] for a coordinate."
INVALID_TIME_INCREASE_ERROR_MSG = "InvalidTimeIncrease: Later coordinates must have a higher time than those before or the robot will not be able to move to the given point."


robot_joint_position = [107.38158881112184, 46.157985403753784, 103.3563681945703, 30.485646401675925, 107.38158881112184, 135.0]
robot_cartesian_position = [280, 132, 74, -90, 135, -90]


class MoveNotPossible(Exception):
    """
    Used for error handling. Also the reason for the inheritance of Exception. The following error is described as the following:

    MoveNotPossible: The given movement command can not be executed by the program. Please make sure that it dies not result in a singularity
    """
    pass


class JointSyntaxError(Exception):
    """
    Used for error handling. Also the reason for the inheritance of Exception. The following error is described as the following:

    JointSyntaxError: The syntax must be [Joint1, joint2, joint3, joint4, joint5, joint6] for a joint
    """
    pass


class TakesOnlyTwoCoordinates(Exception):
    """
    Used for error handling. Also the reason for the inheritance of Exception. The following error is described as the following:

    TakesOnlyTwoCoordinates: This class takes only two coordinates.
    """
    pass


class CoordinateSyntaxError(Exception):
    """
    Used for error handling. Also the reason for the inheritance of Exception. The following error is described as the following:

    CoordinateSyntaxError: The syntax must be: [x, y, z, alpha, beta, gamma, time] for a coordinate.
    """
    pass


class InvalidTimeIncrease(Exception):
    """
    Used for error handling. Also the reason for the inheritance of Exception. The following error is described as the following:

    InvalidTimeIncrease: Later coordinates must have a higher time than those before or the robot will not be able to move to the given point.
    """
    pass


np.set_printoptions(suppress=True)


def joint_matrix(alpha, a, d, theta):
    """
    Function to create transformation matrix.

    :param alpha: alpha value from David Hartenberg parameters.
    :param a: a value from David Hartenberg parameters.
    :param d: d value from David Hartenberg parameters.
    :param theta: theta value from David Hartenberg parameters.
    :return: returns a 4x4 matrix containing the transformation matrix for the given parameters.
    """
    matrix = np.matrix([[m.cos(theta), -m.sin(theta), 0, a],
                        [m.sin(theta) * m.cos(alpha), m.cos(theta) * m.cos(alpha), -m.sin(alpha), -m.sin(alpha) * d],
                        [m.sin(theta) * m.sin(alpha), m.cos(theta) * m.sin(alpha), m.cos(alpha), m.cos(alpha) * d],
                        [0, 0, 0, 1]])

    return matrix


def get_rot_x_matrix(angle):
    """
    Get rotation matrix around x axis.
    :param angle: Angle the rotation matrix should rotate an object.
    :return: Returns the rotation matrix.
    """
    rot_x_matrix = np.matrix([[1, 0, 0, 0],
                              [0, m.cos(angle), -m.sin(angle), 0],
                              [0, m.sin(angle), m.cos(angle), 0],
                              [0, 0, 0, 1]])

    return rot_x_matrix


def get_rot_y_matrix(angle):
    """
    Get rotation matrix around y axis.
    :param angle: Angle the rotation matrix should rotate an object.
    :return: Returns the rotation matrix.
    """
    rot_y_matrix = np.matrix([[m.cos(angle), 0, m.sin(angle), 0],
                              [0, 1, 0, 0],
                              [-m.sin(angle), 0, m.cos(angle), 0],
                              [0, 0, 0, 1]])

    return rot_y_matrix


def get_rot_z_matrix(angle):
    """
    Get rotation matrix around z axis.
    :param angle: Angle the rotation matrix should rotate an object.
    :return: Returns the rotation matrix.
    """
    rot_z_matrix = np.matrix([[m.cos(angle), -m.sin(angle), 0, 0],
                              [m.sin(angle), m.cos(angle), 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

    return rot_z_matrix


def orientation_degree_to_radians(orientation):
    """
    Converts angle from degree to radians
    :param orientation: Angle in degree.
    :return: Angle in radians
    """
    for angle_index in range(len(orientation)):
        orientation[angle_index] = orientation[angle_index] / 180 * m.pi

    return orientation


def radians_to_degree(angle):
    """
    Converts angle from radians to degree.
    :param angle: Angle in radians.
    :return: Angle in degree.
    """
    angle = angle / m.pi * 180

    return angle


def radians_to_degree_list(list_radians):
    """
    Converts radians in a list to degree
    :param list_radians: List containing angles in radians
    :return: List containing angles in degrees
    """
    response = []

    for angle in list_radians:
        response.append(angle / m.pi * 180)

    return response


def transform_list_into_range(untransformed_list, min_value, max_value, adjuster):
    """
    Takes a list containing angles and checks if they are in a given range.
    :param untransformed_list: List containing angles
    :param min_value: Min value in range
    :param max_value: Max value in range
    :param adjuster: How much the values may be adjusted
    :return: Transformed list. Some of the elements may have been changed to False if the given value is not in the range.
    """
    for element_index in range(len(untransformed_list)):
        if not untransformed_list[element_index]:
            continue

        while untransformed_list[element_index] < min_value:
            untransformed_list[element_index] += adjuster
            if untransformed_list[element_index] > max_value:
                untransformed_list[element_index] = False

        while untransformed_list[element_index] > max_value:
            untransformed_list[element_index] -= adjuster
            if untransformed_list[element_index] < min_value:
                untransformed_list[element_index] = False
    return untransformed_list


def best_end_joint(current_joint, joints):
    """
    Takes the current joint position and a list containing of possible joint positions and returns the closest possible joint position.
    :param current_joint: The current position of the robots joint.
    :param joints: The possible joint positions for a point.
    :return: The closest joint position to the current joint position
    """
    lowest_score_joint = []
    lowest_score_value = -1

    if len(joints) == 0:
        raise MoveNotPossible(MOVE_NOT_POSSIBLE_ERROR_MSG)

    for joint in joints:
        angle_difference = 0

        for angle_index in range(len(joint)):
            angle_difference += abs(current_joint[angle_index] - joint[angle_index])

        if angle_difference < lowest_score_value or lowest_score_value < 0:
            lowest_score_value = angle_difference
            lowest_score_joint = joint

    return lowest_score_joint


class Kinematics:
    """
    Class to compute the inverse kinematics.
    """
    def __init__(self):
        """
        Constructor for class
        """
        self.robot_controller_settings = RobotControllerSettings()
        self.dh_value_alpha = self.robot_controller_settings.dh_values["alpha"]
        self.dh_value_a = self.robot_controller_settings.dh_values["a"]
        self.dh_value_d = self.robot_controller_settings.dh_values["d"]
        self.dh_value_theta = self.robot_controller_settings.dh_values["theta"]

    def get_joint_values(self, coordinate, orientation):
        """
        Method for getting the joint values from a coordinate and orientation.
        :param coordinate: Coordinate in the format [x, y, z].
        :param orientation: Orientation in the format [roll, pitch, yaw].
        :return: Returns a list containing all possible joint configurations for the robot.
        """
        orientation = orientation_degree_to_radians(orientation)

        rot_x_matrix = get_rot_x_matrix(orientation[0])
        rot_y_matrix = get_rot_y_matrix(orientation[1])
        rot_z_matrix = get_rot_z_matrix(orientation[2])

        wrist_rot_x_matrix = get_rot_x_matrix(0)
        wrist_rot_y_matrix = get_rot_y_matrix(0)
        wrist_rot_z_matrix = get_rot_z_matrix(0)

        wrist = wrist_rot_z_matrix * wrist_rot_y_matrix * wrist_rot_x_matrix

        wrist[0, 3] = 0
        wrist[1, 3] = 0
        wrist[2, 3] = 66

        T_link0_wrist = rot_z_matrix * rot_y_matrix * rot_x_matrix
        T_link0_wrist[0, 3] = coordinate[0]
        T_link0_wrist[1, 3] = coordinate[1]
        T_link0_wrist[2, 3] = coordinate[2]

        T_link0_link6 = T_link0_wrist * inv(wrist)

        theta_1 = self.kin_calculate_theta_1(T_link0_link6)  # Contains two angles

        # For calculating theta_4, theta_5 and theta_6 a rotation matrix consisting of theta_1 and the other angles is needed. This is created:
        rot_z_matrix_theta_1_1 = get_rot_z_matrix(theta_1[0] + self.dh_value_theta[0])
        rot_z_matrix_theta_1_2 = get_rot_z_matrix(theta_1[1] + self.dh_value_theta[0])

        rot_matrix_alpha_link_1 = get_rot_x_matrix(self.dh_value_alpha[0])
        rot_matrix_alpha_link_2 = get_rot_x_matrix(self.dh_value_alpha[1])
        rot_matrix_alpha_link_3 = get_rot_x_matrix(self.dh_value_alpha[2])

        rot_matrix_theta_link_2 = get_rot_z_matrix(self.dh_value_theta[1])
        rot_matrix_theta_link_3 = get_rot_z_matrix(self.dh_value_theta[2])

        rot_matrix_link_1_theta_1_1_inv = inv(rot_matrix_alpha_link_1 * rot_z_matrix_theta_1_1)
        rot_matrix_link_1_theta_1_2_inv = inv(rot_matrix_alpha_link_1 * rot_z_matrix_theta_1_2)
        rot_matrix_link_2_inv = inv(rot_matrix_alpha_link_2 * rot_matrix_theta_link_2)
        rot_matrix_link_3_inv = inv(rot_matrix_alpha_link_3 * rot_matrix_theta_link_3)

        rot_matrix_link_1_to_link_3_theta_1_1_inv = rot_matrix_link_3_inv * rot_matrix_link_2_inv * rot_matrix_link_1_theta_1_1_inv
        rot_matrix_link_1_to_link_3_theta_1_2_inv = rot_matrix_link_3_inv * rot_matrix_link_2_inv * rot_matrix_link_1_theta_1_2_inv

        rot_matrix_theta_1_1 = rot_matrix_link_1_to_link_3_theta_1_1_inv * rot_z_matrix * rot_y_matrix * rot_x_matrix
        rot_matrix_theta_1_2 = rot_matrix_link_1_to_link_3_theta_1_2_inv * rot_z_matrix * rot_y_matrix * rot_x_matrix

        # Based on the rotational matrix, it is possible to get the angle theta5 using the inverse cosinus to field
        # (3,3) in the matrix. However due to the nature of cosinus to a point, it will not be possible to estimate if
        # the angle is positive or negative.

        theta_5 = self.kin_calculate_theta_5(rot_matrix_theta_1_1) + self.kin_calculate_theta_5(rot_matrix_theta_1_2)

        theta_6_theta_1_1 = [self.kin_calculate_theta_6(rot_matrix_theta_1_1, theta_5[0]),
                             self.kin_calculate_theta_6(rot_matrix_theta_1_1, theta_5[1])]
        theta_6_theta_1_2 = [self.kin_calculate_theta_6(rot_matrix_theta_1_2, theta_5[2]),
                             self.kin_calculate_theta_6(rot_matrix_theta_1_2, theta_5[3])]
        theta_6 = theta_6_theta_1_1 + theta_6_theta_1_2

        # Moving the

        pre_theta_4_theta_1_1 = [self.kin_calculate_theta_4(rot_matrix_theta_1_1, theta_5[0]),
                                 self.kin_calculate_theta_4(rot_matrix_theta_1_1, theta_5[1])]
        pre_theta_4_theta_1_2 = [self.kin_calculate_theta_4(rot_matrix_theta_1_2, theta_5[2]),
                                 self.kin_calculate_theta_4(rot_matrix_theta_1_2, theta_5[3])]
        pre_theta_4 = pre_theta_4_theta_1_1 + pre_theta_4_theta_1_2

        # Currently theta5 and theta6 consists of 4 different angles whereas theta1 consists of 2 angles.
        # To get each angle to match in the angle, theta1 will be defined as:

        theta_1 = [theta_1[0], theta_1[0], theta_1[1], theta_1[1]]

        # This gives the possibility to make a loop of 4 as each array will consist of this:

        inverse_kinematics_list = []
        theta_1 = transform_list_into_range(theta_1, MIN_ANGLE_LINK_1_5, MAX_ANGLE_LINK_1_5, 2 * m.pi)
        theta_5 = transform_list_into_range(theta_5, MIN_ANGLE_LINK_1_5, MAX_ANGLE_LINK_1_5, 2 * m.pi)
        theta_6 = transform_list_into_range(theta_6, MIN_ANGLE_LINK_6, MAX_ANGLE_LINK_6, 2 * m.pi)

        for i in range(4):
            # Checking if any of them are out of range (Joint space more specifically).
            if not theta_1[i] or not theta_5[i] or not theta_6[i]:
                continue

            T_link0_link1_rotation = joint_matrix(self.dh_value_alpha[0], self.dh_value_a[0], self.dh_value_d[0],
                                                  self.dh_value_theta[0] + theta_1[i])
            T_link4_rotation_link4 = joint_matrix(0, 0, self.dh_value_d[3], 0)
            T_link4_link5 = joint_matrix(self.dh_value_alpha[4], self.dh_value_a[4], self.dh_value_d[4],
                                         self.dh_value_theta[4] + theta_5[i])
            T_link5_link6 = joint_matrix(self.dh_value_alpha[5], self.dh_value_a[5], self.dh_value_d[5],
                                         self.dh_value_theta[5] + theta_6[i])

            T_link1_link3 = inv(T_link0_link1_rotation) * T_link0_link6 * inv(T_link5_link6) * inv(T_link4_link5) * inv(
                T_link4_rotation_link4)

            # Sorting points which are further away than the robot can reach.
            if m.sqrt(T_link1_link3[0, 3] ** 2 + T_link1_link3[2, 3] ** 2) > self.dh_value_a[2] + self.dh_value_a[3]:
                continue

            theta_2_theta_3 = self.kin_calculate_theta_2_3(T_link1_link3)

            theta_2_untransformed = theta_2_theta_3[0]
            theta_3_untransformed = theta_2_theta_3[1]

            theta_2 = transform_list_into_range(theta_2_untransformed, MIN_ANGLE_LINK_1_5, MAX_ANGLE_LINK_1_5, 2 * m.pi)
            theta_3 = transform_list_into_range(theta_3_untransformed, MIN_ANGLE_LINK_1_5, MAX_ANGLE_LINK_1_5, 2 * m.pi)

            theta_4_untransformed = [False, False]

            for angle_index in range(2):
                if theta_2[angle_index] and theta_3[angle_index]:
                    theta_4_untransformed[angle_index] = pre_theta_4[i] - theta_2[angle_index] - theta_3[angle_index]

            theta_4 = transform_list_into_range(theta_4_untransformed, MIN_ANGLE_LINK_1_5, MAX_ANGLE_LINK_1_5, 2 * m.pi)

            # Creates the list.
            for j in range(2):
                # Checking if any of them are out of range (Joint space more specifically).
                if not theta_2[j] or not theta_3[j] or not theta_4[j]:
                    continue

                inverse_kinematics_list.append(
                    radians_to_degree_list([theta_1[i], theta_2[j], theta_3[j], theta_4[j], theta_5[i], theta_6[i]]))

        return inverse_kinematics_list

    # Returns two angles.
    def kin_calculate_theta_1(self, T_link0_link6):
        """
        Calculates theta 1
        :param T_link0_link6: Transformation matrix from link 1 to link 6.
        :return: Returns the two possible values for theta 1
        """
        T_link6_rotation_link6 = joint_matrix(0, 0, self.dh_value_d[5], 0)

        T_link0_link6_rotation = T_link0_link6 * inv(T_link6_rotation_link6)

        x_6 = T_link0_link6_rotation[0, 3]
        y_6 = T_link0_link6_rotation[1, 3]

        theta_1_1 = (m.acos(self.dh_value_d[3] / ((x_6 ** 2 + y_6 ** 2) ** 0.5)) + m.atan2(y_6, x_6))
        theta_1_2 = (-m.acos(self.dh_value_d[3] / ((x_6 ** 2 + y_6 ** 2) ** 0.5)) + m.atan2(y_6, x_6))

        return [theta_1_1, theta_1_2]

    def kin_calculate_theta_2_3(self, T_link2_link3):
        """
        Calculates theta 2 and 3
        :param T_link2_link3: Transformation matrix from link 2 to link 3
        :return: Returns the possible configurations for theta 2 and theta 3.
        """
        x = abs(T_link2_link3.item((0, 3)))
        z = T_link2_link3.item((2, 3))

        mirror_angle_length = m.sqrt(x ** 2 + z ** 2)

        direction = -1 * abs(T_link2_link3.item((0, 3))) / T_link2_link3.item((0, 3))

        theta_2_1 = direction * (m.acos(z / mirror_angle_length) + m.acos(
            (self.dh_value_a[3] ** 2 - self.dh_value_a[2] ** 2 - mirror_angle_length ** 2) / (
                    2 * self.dh_value_a[2] * mirror_angle_length)) + m.pi)
        theta_3_1 = direction * (
            m.acos((mirror_angle_length ** 2 - self.dh_value_a[3] ** 2 - self.dh_value_a[2] ** 2) / (
                    2 * self.dh_value_a[2] * self.dh_value_a[3])))

        theta_2_2 = direction * (m.acos(z / mirror_angle_length) - m.acos(
            (self.dh_value_a[3] ** 2 - self.dh_value_a[2] ** 2 - mirror_angle_length ** 2) / (
                    2 * self.dh_value_a[2] * mirror_angle_length)) - m.pi)
        theta_3_2 = -1 * direction * (
            m.acos((mirror_angle_length ** 2 - self.dh_value_a[3] ** 2 - self.dh_value_a[2] ** 2) / (
                    2 * self.dh_value_a[2] * self.dh_value_a[3])))

        theta_2 = [theta_2_1, theta_2_2]
        theta_3 = [theta_3_1, theta_3_2]

        return theta_2, theta_3

    def kin_calculate_theta_4(self, rot_matrix, theta_5):
        """
        Calculates theta 4
        :param rot_matrix: Rotation matrix describing rotation from link 4 to link 6.
        :param theta_5: Value for theta 5
        :return: Returns the value for theta 4
        """
        # -self.dh_value_theta[3] is subtracted due to the calculated angle is based on theta2+theta3+theta4+self.dh_value_theta4 = calculated angle.
        theta_4 = m.atan2(rot_matrix.item((1, 2)) * m.sin(theta_5), rot_matrix.item((0, 2)) * m.sin(theta_5)) - \
                  self.dh_value_theta[3]

        return theta_4

    def kin_calculate_theta_5(self, rot_matrix):
        """
        Calculates theta 5
        :param rot_matrix: Rotation matrix describing rotation from link 4 to link 6.
        :return: Returns the two values for theta 5
        """
        theta_5_unsigned = m.acos(rot_matrix.item((2, 2)))

        theta_5 = [theta_5_unsigned, -theta_5_unsigned]

        return theta_5

    def kin_calculate_theta_6(self, rot_matrix, theta_5):
        """
         Calculates theta 6
         :param rot_matrix: Rotation matrix describing rotation from link 4 to link 6.
         :param theta_5: Value for theta 5
         :return: Returns the value for theta 6
         """
        theta_6 = m.atan2(rot_matrix.item((2, 1)) * m.sin(theta_5), -1 * rot_matrix.item((2, 0)) * m.sin(theta_5))

        return theta_6


class MoveJ(Kinematics):
    """
    Class for moving in joint space
    """
    def __init__(self, robot_pointer, positions_with_time):
        """
        Constructor for the MoveJ class. Takes the robot and position.
        :param robot_pointer: A pointer to the robot class for the robot
        :param positions_with_time: The position with time.
        """
        self.robot = robot_pointer
        self.pos_w_time = positions_with_time
        self.start_joint_values = robot_joint_position

        start_time = 0

        if not len(self.start_joint_values) == 6:
            raise JointSyntaxError(JOINT_SYNTAX_ERROR_MSG)

        for coordinate in self.pos_w_time:
            if not len(coordinate) == 7:
                raise CoordinateSyntaxError(COORDINATE_SYNTAX_ERROR_MSG)

            if coordinate[6] <= start_time:
                raise InvalidTimeIncrease(INVALID_TIME_INCREASE_ERROR_MSG)

            start_time = coordinate[6]

        super().__init__()

        self.joint_matrix = self.convert_coordinates_to_joint_movements()

    def convert_coordinates_to_joint_movements(self):
        """
        Method that converts the coordinates into joint degrees for each via point and end position.
        :return: Returns the transformed matrix.
        """
        start_joint = self.start_joint_values
        joint_values = []

        for position_index in range(len(self.pos_w_time)):
            position = self.pos_w_time[position_index]

            coordinate = position[0:3]
            orientation = position[3:6]
            time_val = position[6]

            joints = self.get_joint_values(coordinate, orientation)

            if position_index == 0:
                joint_values.append([start_joint, 0])

                start_joint = best_end_joint(start_joint, joints)
                joint_values.append([start_joint, time_val])

            else:
                start_joint = best_end_joint(start_joint, joints)
                joint_values.append([start_joint, time_val])

        return joint_values

    def move_joint(self):
        """
        Method for moving the robot in the joint space thrugh the different via points.
        """
        global robot_joint_position, robot_cartesian_position

        self.joint_matrix = self.convert_coordinates_to_joint_movements()

        end_joint_position = []

        for move_index in range(
                len(self.joint_matrix) - 1):  # The reason for -1 is because the function is indexing 1 ahead.
            time_val = self.joint_matrix[move_index][1]
            time_val_plus_1 = self.joint_matrix[move_index + 1][1]

            joint_movement_time = time_val_plus_1 - time_val

            joint_function_values = []

            for joint_index in range(len(self.joint_matrix[move_index][0])):
                v_start = 0
                v_end = 0

                joint_val = self.joint_matrix[move_index][0][joint_index]
                joint_val_plus_1 = self.joint_matrix[move_index + 1][0][joint_index]

                if not move_index == 0:
                    joint_val_minus_1 = self.joint_matrix[move_index - 1][0][joint_index]
                    time_val_minus_1 = self.joint_matrix[move_index - 1][1]

                    v_start = ((joint_val - joint_val_minus_1) / (time_val - time_val_minus_1) + (
                                (joint_val_plus_1 - joint_val) / (time_val_plus_1 - time_val))) / 2

                if not move_index == len(self.joint_matrix) - 2:
                    joint_val_plus_2 = self.joint_matrix[move_index + 2][0][joint_index]
                    time_val_plus_2 = self.joint_matrix[move_index + 2][1]

                    v_end = ((joint_val_plus_1 - joint_val) / (time_val_plus_1 - time_val) + (
                                (joint_val_plus_2 - joint_val_plus_1) / (time_val_plus_2 - time_val_plus_1))) / 2

                a_0 = joint_val
                a_1 = v_start
                a_2 = 3 / (joint_movement_time ** 2) * (
                            joint_val_plus_1 - joint_val) - 2 / joint_movement_time * v_start - 1 / joint_movement_time * v_end
                a_3 = (-2) / (joint_movement_time ** 3) * (joint_val_plus_1 - joint_val) + 1 / (
                            joint_movement_time ** 2) * (v_end + v_start)

                joint_function_values.append([a_0, a_1, a_2, a_3])

            for freq in range(round(joint_movement_time * self.robot_controller_settings.update_frequency)):
                joints_value = []
                for fp in joint_function_values:  # fp: function parameters
                    x = freq / self.robot_controller_settings.update_frequency

                    function_value = fp[0] + fp[1] * x + fp[2] * x ** 2 + fp[3] * x ** 3
                    joints_value.append(function_value)

                self.robot.set_joints(joints_value)
                end_joint_position = joints_value
                t.sleep(1 / self.robot_controller_settings.update_frequency)

        robot_joint_position = end_joint_position
        robot_cartesian_position = self.pos_w_time[-1][:6]


# IKKE OPDATERET UD FRA NYESTE EXCEPTIONS!
class MoveL(MoveJ):
    """
    Class for moving in cartesian space.
    """
    def __init__(self, robot_pointer, positions_with_time):
        """
        Constructor for MoveL class
        :param robot_pointer: Pointer to robot controller
        :param positions_with_time: Position for where the robot should move to.
        """
        self.robot = robot_pointer
        self.pos_w_time_linear = positions_with_time
        self.move_time = self.pos_w_time_linear[0][6]

        super().__init__(self.robot, self.pos_w_time_linear)

        if not len(positions_with_time) == 1:
            raise TakesOnlyTwoCoordinates(TAKES_ONLY_TWO_COORDINATES_ERROR_MSG)

        start_time = 0

        for coordinate in self.pos_w_time_linear:
            if not len(coordinate) == 7:
                raise CoordinateSyntaxError(COORDINATE_SYNTAX_ERROR_MSG)

            if coordinate[6] <= start_time:
                raise InvalidTimeIncrease(INVALID_TIME_INCREASE_ERROR_MSG)

            start_time = coordinate[6]

        self.start_pos_cartesian = robot_cartesian_position

        self.pos_w_time_linear = self.get_linear_via_points()
        self.pos_w_time = self.pos_w_time_linear

    def get_linear_via_points(self):
        """
        Method for getting all via points on the path, the robot moves on.
        :return: Returns the coordinates which will be used as via points in the joint movement.
        """
        linear_movement_coords_w_time = []

        function_values = []

        for i in range(6):
            a_0 = self.start_pos_cartesian[i]
            a_1 = 0
            a_2 = 3 / (self.move_time ** 2) * (self.pos_w_time_linear[0][i] - self.start_pos_cartesian[i])
            a_3 = (-2) / (self.move_time ** 3) * (self.pos_w_time_linear[0][i] - self.start_pos_cartesian[i])

            function_values.append([a_0, a_1, a_2, a_3])

        for i in range(1, round(self.move_time * self.robot_controller_settings.linear_frequency + 1)):

            coordinate = []

            x = i / self.robot_controller_settings.linear_frequency

            for j in range(6):
                coordinate.append(function_values[j][0] + function_values[j][1] * x + function_values[j][2] * x ** 2 + function_values[j][3] * x ** 3)

            coordinate.append(x)

            linear_movement_coords_w_time.append(coordinate)

        return linear_movement_coords_w_time


class Robot:
    """
    Robot class used to talk with RoboDK and the correct robot
    """
    def __init__(self, rdk_link):
        """
        Constructor for class
        """
        self.robot_settings = RobotControllerSettings()

        self.rdk_link = rdk_link
        self.prev_position = [0, 0, 0, 0, 0, 0]
        self.time_between_move = 1 / self.robot_settings.update_frequency
        self.max_joint_speed = 180

    def set_joints(self, position, robot_speed=100):
        """
        Sets the joint values for the robot
        :param position: Position in joint space.
        :param robot_speed: Not used due to it not being neccessary, but has to be there in order to use the class together with other classes.'
        """

        # Prints if one of the joins exceed maximum speed.
        for index in range(6):
            speed = abs((position[index] - self.prev_position[index]) / self.time_between_move)
            if speed > self.max_joint_speed:
                print(f"Joint {index+1} exceeded max speed in simulation at {position}")

        self.prev_position = position

        self.rdk_link.setJoints(position)


class RobotMyCobot:
    """
    Robot class used to talk with MyCobot 320 PI
    """
    def __init__(self, robo_link):
        """
        Constructor for class
        """
        self.robo_link = robo_link

    def set_joints(self, position, robot_speed=100):
        """
        Sets the joint values for the robot
        :param position: Position in joint space.
        :param robot_speed: Value between 0-100 setting the robot's speed.
        """
        encoder_val = []
        encoder_dir = [1, 1, -1, 1, 1, 1]

        for joint_val_index in range(len(position)):
            encoder_val.append(round(2048 - position[joint_val_index] / 90 * 1024 * encoder_dir[joint_val_index]))

        self.robo_link.set_encoders(encoder_val, robot_speed)


class RobotMyCobotAndSim:
    """
    Robot class used to talk with MyCobot 320 PI and RoboDK
    """
    def __init__(self, robo_link, rdk_link):
        """
        Constructor for class
        """
        self.rdk_link = rdk_link
        self.robo_link = robo_link

    def set_joints(self, position, robot_speed=100):
        """
        Sets the joint values for the robot
        :param position: Position in joint space.
        :param robot_speed: Value between 0-100 setting the robot's speed.
        """
        encoder_val = []
        encoder_dir = [1, 1, -1, 1, 1, 1]

        for joint_val_index in range(len(position)):
            encoder_val.append(round(2048 - position[joint_val_index] / 90 * 1024 * encoder_dir[joint_val_index]))

        self.robo_link.set_encoders(encoder_val, robot_speed)

        self.rdk_link.setJoints(position)

