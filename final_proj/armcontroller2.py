import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import numpy as np
import scipy 
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import math
import array
import tf2_ros
from tf2_ros import Buffer, LookupException, TransformListener, TransformException
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, PointStamped
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.node import Node
from pan_tilt_msgs.msg import PanTiltCmdDeg
import numpy as np
import geometry_msgs.msg


class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.pantil_deg_cmd =PanTiltCmdDeg()
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg,"pan_tilt_cmd_deg",10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.br = TransformBroadcaster(self)
        #这里是自定义的类型
        #self.publisher_ = self.create_publisher(geometry_msgs.msg.Pose, 'object_square', 10)
        
        self.cam = self.create_subscription(PoseArray,"/aruco_poses",self.cam2arm,10)
        
        self.read_timer = self.create_timer(0.5,self.control)
        #这个的作用是看是否到位
        self.initial = False
        self.cam_position = []
        self.cam_rotation = []
        self.arm_position =[]
        self.arm_rotation = []
        
        # self.pub_timer = self.create_timer(0.5, self.timers_cb)               

        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        self.cam_pose = PoseArray()
      #  self.tf_reader = TF_Reader()
        
        self.cnt = 0
        self.thred = 0.15
        
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')

        self.reached = False
  
        self.machine_state = "INIT"

        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))
        self.seita = 0.0
        self.pose_stage = 0
        self.attempt = 0
        self.command = None
        self.grasp_state_publisher = self.create_publisher(String, '/grasp_state', 10)
        self.grasp_command_subscription = self.create_subscription(
            String,
            '/grasp_command',
            self.grasp_command_callback,
            10
        )
        #self.grasp_state_publisher.publish(String(data="tograsp"))
        
        pass
    
    def grasp_command_callback(self, msg):
        self.command = msg.data
        print("Received command:", self.command)
       
        

    def js_cb(self, msg):
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])
                
    def go_to_pre_grasp_position(self):
        # 机械臂的预备位置
        # 假设这些角度可以使机械臂的末端抬高并远离障碍物
        pre_grasp_pos = [-1.5, 0.0, -1.3, 0.8]  # 示例关节角度，单位为弧度
        self.initial = self.set_group_pos(pre_grasp_pos)
        # 添加延时确保机械臂到达预备位置
        #time.sleep(5.0)  # 延时5秒，根据实际情况调整
            
    def control(self):
        try:
                if self.pose_stage == 0:
                     self.release()
                     self.go_to_pre_grasp_position() 
                     #当到达目标位置并且接收到导航传递的topic后，切换到pose_stage = 1
                     if self.initial and self.command == "grasp":
                         print("到达A点,开始抓取")
                         self.pose_stage = 1

                if self.pose_stage == 1:
                    #执行抓取操作
                    print(self.pose_stage)
                    self.arm_position =[]
                    #print("a")
                    self.arm_rotation = []
                    #print("b")
                    #trans = self.tf_buffer.lookup_transform("px100/base_link", "object_square", now)
                    try:
                        #now = self.get_clock().now().to_msg()
                        #print("d")
                        now = rclpy.time.Time()
                        #print("c")
                        trans = self.tf_buffer.lookup_transform("px100/base_link", "object_square", now)
                        #print("d")
                        # 处理变换逻辑
                        # 目前报错：未找到变换-LookupException
                        # 实验更新日志：2023/12/23 添加tf broadcaster,尝试发送object_square，存在稳定性问题，仅成功一次
                        # ...

                    except LookupException as ex:
                        print(ex)
                        self.get_logger().error('未找到变换 - LookupException')
                    # except ConnectivityException:
                    #     self.get_logger().error('连接问题 - ConnectivityException')
                    # except ExtrapolationException:
                    #     self.get_logger().error('时间外插异常 - ExtrapolationException')
                    # except Exception as e:
                    #     self.get_logger().error(f'变换查询时发生未知错误: {str(e)}')
                    
                    
                   # trans = self.tf_buffer.lookup_transform("px100/base_link","object_square",now)
                    print("start grasp")
                    #得到相对的转换矩阵
                    trans_position_x = trans.transform.translation.x
                    trans_position_y = trans.transform.translation.y
                    trans_position_z = trans.transform.translation.z
                    
                    self.arm_position.append(trans_position_x)
                    self.arm_position.append(trans_position_y)
                    self.arm_position.append(trans_position_z)
                    
                    
                    self.seita = math.atan(trans_position_y/trans_position_x)
                    
                    trans_rotation_x = trans.transform.rotation.x
                    trans_rotation_y = trans.transform.rotation.y
                    trans_rotation_z = trans.transform.rotation.z
                    trans_rotation_w = trans.transform.rotation.w
                    
                    self.arm_rotation.append(trans_rotation_x)
                    self.arm_rotation.append(trans_rotation_y)
                    self.arm_rotation.append(trans_rotation_z)
                    self.arm_rotation.append(trans_rotation_w)
                    
                    self.transform()
                    max_attempt = 10
                    if self.pose_stage == 2:
                        #while True:
                        while self.attempt < max_attempt:
                            msg = String()
                            msg.data = "torelease"
                            self.grasp_state_publisher.publish(msg)
                            print(self.command)
                            if self.command == "release":
                                print("你在干嘛")
                                break
                            self.attempt += 1
                        if self.attempt == max_attempt:
                            print("达到最大尝试次数，退出循环")
                        print("要到3了")
                        self.pose_stage = 3
                    
                    #当接收到导航的release命令后，放开物体，发布finish状态
                    
                if self.pose_stage == 3:    
                    print(self.pose_stage)
                    print(self.command)
                    if  self.command == "release":
                        release_pose = [-1.5, 0.0, 0.8, -0.5]
                        self.set_group_pos(release_pose)
                        self.release()
                        #self.go_to_pre_grasp_position()
                        while True:
                            msg = String()
                            msg.data = "finish"
                            self.grasp_state_publisher.publish(msg)

        except :
            pass

        pass
    
    def transform(self):
        if len(self.arm_rotation) >= 4 and len(self.cam_rotation) >= 4:
            arm_matrix = self.create_transformation_matrix(self.arm_position, self.arm_rotation[-4:])
            arm_matrix[0,0] = math.cos(self.seita)
            arm_matrix[0,1] = -math.sin(self.seita)
            arm_matrix[0,2] = 0
            
            arm_matrix[1,0] = math.sin(self.seita)
            arm_matrix[1,1] = math.cos(self.seita)
            arm_matrix[1,2] = 0
            
            arm_matrix[2,0] = 0
            arm_matrix[2,1] = 0
            arm_matrix[2,2] = 1 
             
             
            print(arm_matrix)
            cam_matrix = self.create_transformation_matrix(self.cam_position, self.cam_rotation[-4:])           
            new_matrix = np.dot(arm_matrix, cam_matrix) 
            self.matrix_control(arm_matrix)  
            
            if self.reached:
                self.gripper_controller(0.78, 1.0) 
                self.set_group_pos([-1.5, 0.0, -1.3, 0.8])
                self.pose_stage = 2
                
        else:
            print(len(self.arm_rotation), len(self.cam_rotation))
            print("Insufficient rotation data")    
            
   
    
    def cam2arm(self, msg):
             
        self.cam_position = []
        self.cam_rotation = []
        
        camera_x = msg.poses[0].position.x
        camera_y = msg.poses[0].position.y
        camera_z = msg.poses[0].position.z
        
        camera_qx = msg.poses[0].orientation.x
        camera_qy = msg.poses[0].orientation.y
        camera_qz = msg.poses[0].orientation.z
        camera_qw = msg.poses[0].orientation.w
        
        self.cam_position.append(camera_x)
        self.cam_position.append(camera_y)
        self.cam_position.append(camera_z)
        
        self.cam_rotation.append(camera_qx)
        self.cam_rotation.append(camera_qy)
        self.cam_rotation.append(camera_qz)
        self.cam_rotation.append(camera_qw)
        
        #````````````````````````````````````````````````````
        transform_stamped_msg = TransformStamped()
        
        transform_stamped_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        transform_stamped_msg.header.frame_id = "camera_color_optical_frame"  # 替换为实际的相机坐标系
        transform_stamped_msg.child_frame_id = "object_square"  # 替换为新的坐标系名
        transform_stamped_msg.transform.translation.x = camera_x
        transform_stamped_msg.transform.translation.y =  camera_y
        transform_stamped_msg.transform.translation.z =  camera_z
        transform_stamped_msg.transform.rotation.x = camera_qx
        transform_stamped_msg.transform.rotation.y = camera_qy
        transform_stamped_msg.transform.rotation.z = camera_qz
        transform_stamped_msg.transform.rotation.w = camera_qw
        #self.tf_buffer.set_transform(transform_stamped_msg,self.get_name())  
        self.br.sendTransform(transform_stamped_msg) 
    #将position和rotation转化为交换矩阵
    def create_transformation_matrix(self, position, quaternion):
   
        x, y, z, w = quaternion
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z

        rotation_matrix = np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)]
        ])

        transformation_matrix = np.eye(4)  # Create a 4x4 identity matrix
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = position

        return transformation_matrix
    
    

    
    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """
        将旋转矩阵转换为四元数
        """
        # 使用 numpy 的 rotation_matrix_to_quaternion 函数
        quaternion = self.tf_reader.transformations.quaternion_from_matrix(rotation_matrix)
        # 创建 Quaternion 对象
        quaternion_msg = Quaternion()
        quaternion_msg.x = quaternion[0]
        quaternion_msg.y = quaternion[1]
        quaternion_msg.z = quaternion[2]
        quaternion_msg.w = quaternion[3]

        return quaternion_msg

    def set_single_pos(self, name, pos, blocking=True):
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 

        '''
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]
                        cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        cal_name = 'gripper'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos-check_pos)
                        if dis < thred:
                            return True
                        else:
                            print('single joint moving...')
                            return False                       
                    case "gripper":
                        return True

        pass

    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
        else:
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            self.group_pub.publish(self.arm_group_command)
            
            #pos_list[0]和pos_list[1]的范围可能不在-pi到pi之间，使用for循环加上2pi或者减去2pi
            while True:
                for i in range(2):
                    if pos_list[i] > np.pi:
                        pos_list[i] -= 2*np.pi
                        
                    elif pos_list[i] < -np.pi:
                        pos_list[i] += 2*np.pi
                if pos_list[0] <= np.pi and pos_list[0] >= -np.pi and pos_list[1] <= np.pi and pos_list[1] >= -np.pi:
                    break
                    
            
            
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    print('current group pos:', check_pos)
                    if np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= thred:
                            print('waist moving...')
                        if np.abs(pos_list[1] - check_pos[1]) >= thred:
                            print('shoulder moving...')
                        if np.abs(pos_list[2] - check_pos[2]) >= thred:
                            print('elbow moving...')
                        if np.abs(pos_list[3] - check_pos[3]) >= thred:
                            print('wrist moving...')
                            return False            
            pass
        
        
    def is_joint_in_position(self, joint_name, target_pos):
        thred = self.thred
        while True:
            if target_pos > np.pi:
                target_pos -= 2*np.pi
                            
            elif target_pos < -np.pi:
                target_pos += 2*np.pi
            if target_pos <= np.pi and target_pos >= -np.pi:
                break
                
            
            
        joint_index = self.get_joint_index(joint_name)
        if joint_index is not None and len(self.joint_pos) > joint_index:
            if np.abs(target_pos - self.joint_pos[joint_index]) < thred:
                return True
            else:
                print(f'{joint_name} moving...')
        return False
    
    
    def get_joint_index(self, joint_name):
        # 根据关节名称返回关节的索引
        joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']
        if joint_name in joint_names:
            return joint_names.index(joint_name)
        return None
        
    #这个我要先发送waist，然后再发送其他的
    def set_group_pos22(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
      
        # 先移动 waist 关节
        waist_pos = pos_list[0]
        if waist_pos > np.pi:
            waist_pos -= 2 * np.pi
        elif waist_pos < -np.pi:
            waist_pos += 2 * np.pi

        self.set_single_pos('waist', waist_pos+0.09, blocking)
        time.sleep(2)
        # 检查是否到达目标位置
        #给waist一个补偿
        #waist_pos = waist_pos + 0.03
        if blocking:
            if not self.is_joint_in_position('waist', waist_pos+0.09):
                return False
                        
        # 接下来移动其他关节
        other_joints_pos = pos_list[1:]  # 获取除了waist以外的关节位置
        self.arm_group_command.name = "arm"
        self.arm_group_command.cmd = [waist_pos] + other_joints_pos
        self.group_pub.publish(self.arm_group_command)
        time.sleep(2)
        
        if blocking:
            if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    print('current group pos:', check_pos)
                    if np.abs(pos_list[0] - check_pos[0]) < self.thred and np.abs(pos_list[1] - check_pos[1]) < self.thred and np.abs(pos_list[2] - check_pos[2]) < self.thred and np.abs(pos_list[3] - check_pos[3]) < self.thred:
                        return True
                    else:
                        if np.abs(pos_list[0] - check_pos[0]) >= self.thred:
                            print('waist moving...')
                        if np.abs(pos_list[1] - check_pos[1]) >= self.thred:
                            print('shoulder moving...')
                        if np.abs(pos_list[2] - check_pos[2]) >= self.thred:
                            print('elbow moving...')
                        if np.abs(pos_list[3] - check_pos[3]) >= self.thred:
                            print('wrist moving...')
                            return False    
                
           

    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home_pos(self):
        state = self.set_group_pos([0.0, 0.0, 0.0, 0.0])
        return state

    def go_sleep_pos(self):
        state = self.set_group_pos([-1.4, -0.35, 0.7, 1.0])
        return state
    
    
    
    


    def matrix_control(self,T_sd, custom_guess: list[float]=None, execute: bool=True):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.02,
                ev=0.005,
            )
            solution_found = True
            print('success',success, solution_found)
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                print('success',success)
                theta_list = self._wrap_theta_list(theta_list)
                # solution_found = self._check_joint_limits(theta_list)
                solution_found = True
            else:
                solution_found = False

            if solution_found:
                if execute:
                    # if waist_only:
                    #     joint_list = [theta_list[0], 0.0, -1.3, 0.8]
                    # else:
                    #给shoulder一个补偿
                    
                    joint_list = [theta_list[0], theta_list[1]-0.06, theta_list[2], theta_list[3]]
                    
                    self.reached=self.set_group_pos22(joint_list)
                    self.T_sb = T_sd
                return theta_list, True

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        return theta_list, False



    def waist_control(self, pos):
        """
        lower limit = -1.5
        upper limit = 1.5
        """
        pos = float(pos)
        state = self.set_single_pos('waist', pos)
        return state
    
    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = ~0.9
        """
        pos = float(pos)
        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)
        state = self.set_single_pos('elbow', pos)
        return state
    
    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)
        state = self.set_single_pos('wrist_angle', pos)
        return state

    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state

    # 设置夹爪抓取时使用的压力值
    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.
        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    # 打开夹爪
    def release(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    # 控制机械爪进行抓取操作
    def grasp(self, pressure: float = 0.5, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state

    # 确保关节角度在指定范围内
    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        """
        Wrap an array of joint commands to [-pi, pi) and between the joint limits.

        :param theta_list: array of floats to wrap
        :return: array of floats wrapped between [-pi, pi)
        """
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
                #theta_list[x] = self.joint_lower_limits[x]
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
                #theta_list[x] = self.joint_upper_limits[x]
        return theta_list


def main():
    rclpy.init(args=None)
    contoller = ArmController()
    rclpy.spin(contoller)
    rclpy.shutdown()
    
   

if __name__ == '__main__':
    main()
