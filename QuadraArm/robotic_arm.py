from dynamixel_sdk import *
from  utility import *
import numpy as np
import math
from numpy.linalg import inv
from time import sleep
from numpy.linalg import norm
import keyboard

# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PROFILE_VELOCITY       = 112
ADDR_MAX_POSITION_LIMIT     = 48
ADDR_MIN_POSITION_LIMIT     = 52


class Arm4DOF:
    def __init__(self):
        # adjust the following parameters according to your hardware setup
        self.baud_rate = 1000000
        self.max_position_limit = [ 90,  90,  90,  45]  # theta1 ,theta2, theta3, theta4
        self.min_position_limit = [-90, -45, -70, -90]
        self.motor_velocity = 15  # 15 * 0.229 RPM (applied to all motors)
        self.d0 = 6    # cm
        self.d1 = 7.5  # cm
        self.l1 = 6    # cm
        self.l2 = 13   # cm
        self.l3 = 11   # cm
        self.l4 = 14   # cm
        # don't change the following parameters
        self.protocol_version = 2.0
        self.port_handler = None
        self.packet_handler = None

    def connect(self, PortName):
        self.port_handler = PortHandler(PortName)
        self.packet_handler = PacketHandler(self.protocol_version)

        self.open_port()

        if not self.port_handler.setBaudRate(self.baud_rate):
            print("Failed to change the baud rate")

    def open_port(self):
        if not self.port_handler.openPort():
            print("Failed to open the port")

    def close_port(self):
        self.port_handler.closePort()

    def write4byte(self, MotorID, Addr, Data):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, MotorID, Addr, Data)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    def write1byte(self, MotorID, Addr, Data):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, MotorID, Addr, Data)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    def read4byte(self, MotorID, Addr):
        output, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, MotorID, Addr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        return output

    def read1byte(self, MotorID, Addr):
        output, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, MotorID, Addr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))
        return output

    def torques(self, Motor1, Motor2, Motor3, Motor4):
        self.write1byte(1, ADDR_TORQUE_ENABLE, Motor1)
        self.write1byte(2, ADDR_TORQUE_ENABLE, Motor2)
        self.write1byte(3, ADDR_TORQUE_ENABLE, Motor3)
        self.write1byte(4, ADDR_TORQUE_ENABLE, Motor4)

    def init(self):
        self.torques(0, 0, 0, 0)

        max_position = []
        min_position = []
        epsilon = 0.1

        max_position.append(int(11.570 * (self.max_position_limit[0] + epsilon) + 2047.50))
        min_position.append(int(11.570 * (self.min_position_limit[0] - epsilon) + 2047.50))

        max_position.append(int(11.590 * (self.max_position_limit[1] + epsilon) + 2048.70))
        min_position.append(int(11.590 * (self.min_position_limit[1] - epsilon) + 2048.70))

        max_position.append(int(-11.640 * (self.min_position_limit[2] - epsilon) + 2047.50))
        min_position.append(int(-11.640 * (self.max_position_limit[2] + epsilon) + 2047.50))

        max_position.append(int(11.655 * (self.max_position_limit[3] + epsilon) + 2049.95))
        min_position.append(int(11.655 * (self.min_position_limit[3] - epsilon) + 2049.95))

        for i in range(1, 5):
            self.write4byte(i, ADDR_PROFILE_VELOCITY, self.motor_velocity)
            self.write4byte(i, ADDR_MAX_POSITION_LIMIT, max_position[i-1])
            self.write4byte(i, ADDR_MIN_POSITION_LIMIT, min_position[i-1])

        self.torques(1, 1, 1, 1)

    def go_to_thetas(self, thetas):
        values = []
        values.append(int( 11.570 * thetas[0] + 2047.50))
        values.append(int( 11.590 * thetas[1] + 2048.70))
        values.append(int(-11.640 * thetas[2] + 2047.50))
        values.append(int( 11.655 * thetas[3] + 2049.95))

        self.write4byte(1, ADDR_GOAL_POSITION, values[0])
        self.write4byte(2, ADDR_GOAL_POSITION, values[1])
        self.write4byte(3, ADDR_GOAL_POSITION, values[2])
        self.write4byte(4, ADDR_GOAL_POSITION, values[3])

        return self.forward_kinematic(thetas)

    def go_to_home(self):
        self.go_to_thetas([0, 0, 0, 0])

    def read_encoders(self):
        result = []
        for i in range(1, 5):
            result.append(self.read4byte(i, ADDR_PRESENT_POSITION))
        return result

    def read_thetas(self):
        thetas = []
        for i in range(1, 5):
            thetas.append(self.read4byte(i, ADDR_PRESENT_POSITION))

        thetas[0] = int( 0.0864 * thetas[0] - 176.90)
        thetas[1] = int( 0.0863 * thetas[1] - 176.89)
        thetas[2] = int(-0.0859 * thetas[2] + 175.88)
        thetas[3] = int( 0.0858 * thetas[3] - 175.80)
        return thetas

    def forward_kinematic(self, thetas):
        DH = get_DH(thetas, self.d0, self.d1, self.l1, self.l2, self.l3, self.l4)
        T_i_0 = np.eye(4)
        for i in range(5):
            T_i_0 = T_i_0 @ T_i_to_i_minus_1(theta_i=DH[i, 0], d_i=DH[i, 1], r_i=DH[i, 2], alpha_i=DH[i, 3])

        EE = np.array([[0], [0], [0], [1]])
        EE = T_i_0 @ EE
        EE = np.round(EE, 3)
        return [EE[0, 0], EE[1, 0], EE[2, 0]]

    def inverse_kinematic(self, EE):
        x_ee = EE[0] + self.d0
        y_ee = EE[1]
        z_ee = EE[2]

        theta1 = atan2d(y_ee, x_ee)

        x_ee = x_ee - self.l4 * cosd(theta1)
        y_ee = y_ee - self.l4 * sind(theta1)

        r = math.sqrt(pow(x_ee, 2) + pow(y_ee, 2))
        s = z_ee - (self.d1 + self.l1)

        tmp = ((pow(r, 2) + pow(s, 2)) - (pow(self.l2, 2) + pow(self.l3, 2))) / (2 * self.l2 * self.l3)
        if (tmp < -1) or (1 < tmp):
            print("E.E. is out of range, please choose a point inside the workspace")
            return None, None
        tmp = acosd(tmp)
        theta3_1 = +tmp
        theta3_2 = -tmp

        theta2_1 = atan2d(s, r) - atan2d(self.l3 * sind(theta3_1), self.l2 + self.l3 * cosd(theta3_1))
        theta2_2 = atan2d(s, r) - atan2d(self.l3 * sind(theta3_2), self.l2 + self.l3 * cosd(theta3_2))

        theta4_1 = -(theta2_1 + theta3_1)
        theta4_2 = -(theta2_2 + theta3_2)

        ans1 = [round(theta1, 3), round(theta2_1, 3), round(theta3_1, 3), round(theta4_1, 3)]
        ans2 = [round(theta1, 3), round(theta2_2, 3), round(theta3_2, 3), round(theta4_2, 3)]
        return ans1, ans2

    def _check_limits(self, thetas):
        for i in range(4):
            if thetas[i] < self.min_position_limit[i] or self.max_position_limit[i] < thetas[i]:
                return False
        return True

    def go_to_positions(self, EE):
        ans1, ans2 = self.inverse_kinematic(EE)
        if ans1 and ans2:
            if self._check_limits(ans2):
                self.go_to_thetas(ans2)
                return ans2
            elif self._check_limits(ans1):
                self.go_to_thetas(ans1)
                return ans1
            else:
                print("answers are out of limits")
                return None

    def _Jacobian(self, q):
        # q => degree
        def k(alpha1, alpha2, alpha3):
            l2 = 12
            l3 = 12
            l4 = 15
            result = 0
            if alpha1 == 'c':
                result += l2 * cosd(q[1])
            elif alpha1 == 's':
                result += l2 * sind(q[1])

            if alpha2 == 'c':
                result += l3 * cosd(q[1] + q[2])
            elif alpha2 == 's':
                result += l3 * sind(q[1] + q[2])

            if alpha3 == 'c':
                result += l4 * cosd(q[1] + q[2] + q[3])
            elif alpha3 == 's':
                result += l4 * sind(q[1] + q[2] + q[3])

            return result

        J = np.array([
        [-sind(q[0])*k('c','c','c'),-cosd(q[0])*k('s','s','s'),-cosd(q[0])*k(0,'s','s'),-cosd(q[0])*k(0,0,'s')],
        [ cosd(q[0])*k('c','c','c'),-sind(q[0])*k('s','s','s'),-sind(q[0])*k(0,'s','s'),-sind(q[0])*k(0,0,'s')],
        [             0            ,            k('c','c','c'),            k(0,'c','c'),            k(0,0,'c')],
        ])
        return J

    def direct_velocity_kinematic(self, delta_q, q0):
        delta_q = np.array([delta_q]).T
        # degree to radian
        delta_q = delta_q * (np.pi / 180)

        delta_x = self._Jacobian(q0) @ delta_q
        delta_x = np.round(delta_x, 3)
        return [delta_x[0, 0], delta_x[1, 0], delta_x[2, 0]]

    def inverse_velocity_kinematic(self, delta_x, q0):
        delta_x = np.array([delta_x]).T

        J_psudo_inv = self._Jacobian(q0).T @ inv(self._Jacobian(q0) @ self._Jacobian(q0).T)

        delta_q = J_psudo_inv @ delta_x

        # radian to degree
        delta_q = delta_q * (180 / np.pi)

        delta_q = np.round(delta_q, 3)
        return [delta_q[0, 0], delta_q[1, 0], delta_q[2, 0], delta_q[3, 0]]

    def go_in_direct_path(self, q_start, x_end, alpha, epsilon=1, delay_1cm=0.05):
        qc = q_start
        xc = self.forward_kinematic(qc)
        delta_x = np.array(subtract_lists(x_end, xc))
        i = 0
        while norm(delta_x) > epsilon:
            dx = alpha * (delta_x / norm(delta_x))
            dx = dx.tolist()
            dq = self.inverse_velocity_kinematic(dx, qc)
            qc = add_lists(qc, dq)
            xc = self.forward_kinematic(qc)
            delta_x = np.array(subtract_lists(x_end, xc))

            self.go_to_thetas(qc)

            # adaptive delay (wait until the last step is completed)
            # thetas_old = [0, 0, 0, 0]
            # while True:
            #     thetas_cur = self.read_thetas()
            #     d_thetas = np.array(subtract_lists(thetas_cur, thetas_old))
            #     if norm(d_thetas) < 0.1:
            #         break
            #     thetas_old = thetas_cur
            #     sleep(0.05)

            sleep(alpha * delay_1cm)

            i += 1
            print(f"step: {i}, error: {round(norm(delta_x), 3)}")

    def record_path(self, sample_rate):
        path_list = []
        while True:
            path_list.append(self.read_thetas())
            sleep(1 / sample_rate)
            if keyboard.is_pressed("ctrl+q"):
                return path_list

    def travel_path(self, path_list, sample_rate, dilation):
        for theta in path_list:
            self.go_to_thetas(theta)
            sleep(1 / (sample_rate * dilation))

