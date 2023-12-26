from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
import time

class MyNavigator(BasicNavigator):
    def __init__(self):
        super().__init__("MyNavigator")
        self.grasp_state = None
        self.grasp_command_publisher = self.create_publisher(String, '/grasp_command', 10)
        self.grasp_state_subscription = self.create_subscription(String, '/grasp_state',self.grasp_state_callback, 10)
        
        
    
    def navigate_to_pose(self, pose):
        navigator.goToPose(pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()
                    
        rclpy.spin_once(navigator)  # 刷新ROS 2回调
        
        
        result = navigator.getResult()
        return result == TaskResult.SUCCEEDED


            
    def grasp_state_callback(self,msg):
        self.grasp_state = msg.data
        print(msg.data)
        
    # def main(self):
    #     navigator = MyNavigator()
    #     # 设置初始位姿
    #     initial_pose = PoseStamped()
    #     initial_pose.header.frame_id = 'map'
    #     initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    #     initial_pose.pose.position.x = 0.27
    #     initial_pose.pose.position.y = -0.38 
    #     initial_pose.pose.orientation.z = -0.7071
    #     initial_pose.pose.orientation.w = 0.7071


    #     navigator.setInitialPose(initial_pose)
    #     navigator.lifecycleStartup()
    #     navigator.waitUntilNav2Active()

    #     # 定义A点和B点
    #     a_pose = PoseStamped()
    #     a_pose.header.frame_id = 'map'
    #     a_pose.header.stamp = navigator.get_clock().now().to_msg()
    #     a_pose.pose.position.x = 0.32
    #     a_pose.pose.position.y = -3.2
    #     a_pose.pose.orientation.z = -0.08
    #     a_pose.pose.orientation.w = 0.98
        
    #     # a_pose.pose.orientation.z = -0.7071
    #     # a_pose.pose.orientation.w = 0.7071

    #     b_pose = PoseStamped()
    #     b_pose.header.frame_id = 'map'
    #     b_pose.header.stamp = navigator.get_clock().now().to_msg()
    #     b_pose.pose.position.x = 2.7
    #     b_pose.pose.position.y = -3.2
    #     b_pose.pose.orientation.z = 0.7
    #     b_pose.pose.orientation.w = 0.79

    #     # 从S点到A点
        
    #     if self.navigate_to_pose(a_pose):
    #             print("成功到达A点")
    #             #发布grasp_command = "grasp"，抓取物体并等待torelease
    #             #一直发送这个命令，直到grasp_state = "torelease"

    #             while True:
    #                 rclpy.spin_once(navigator)
    #                 grasp_msg = String()
    #                 grasp_msg.data = "grasp"
    #                 self.grasp_command_publisher.publish(grasp_msg)
                    
    #                 print(self.grasp_state)
    #                 if self.grasp_state == "torelease":
    #                     break        

                
    #             # grasp_msg = String()
    #             # grasp_msg.data = "grasp"
    #             # self.grasp_command_publisher.publish(grasp_msg)
    #     else:
    #             print("导航到A点失败")

    #     # 从A点到B点
        
    #     # while True:
    #     #     rclpy.spin_once(navigator)
    #     #     if self.grasp_state == "torelease":
    #     #         break 
        
        
    #     if self.navigate_to_pose(b_pose):
    #         #发布grasp_command = "release"，放下物体并等待finish
    #         while True:
    #             rclpy.spin_once(navigator)
    #             grasp_msg = String()
    #             grasp_msg.data = "release"
    #             self.grasp_command_publisher.publish(grasp_msg)
    #             if self.grasp_state == "finish":
    #                 break
    #         print("导航到B点并成功放下物体")

        
    #     else:
    #         print("导航到B点失败")

    #     # 从B点返回S点
    #     while True:
    #         rclpy.spin_once(navigator)
    #         if self.grasp_state == "finish":
    #             break

    #     if  self.navigate_to_pose(initial_pose):
    #         print("成功返回起点")
    #     else:
    #         print("返回起点失败")

    #     rclpy.shutdown()
        
if __name__ == '__main__':
    rclpy.init()
    navigator = MyNavigator()
    # 设置初始位姿
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.27
    initial_pose.pose.position.y = -0.38 
    initial_pose.pose.orientation.z = -0.7071
    initial_pose.pose.orientation.w = 0.7071


    navigator.setInitialPose(initial_pose)
    navigator.lifecycleStartup()
    navigator.waitUntilNav2Active()

    # 定义A点和B点
    a_pose = PoseStamped()
    a_pose.header.frame_id = 'map'
    a_pose.header.stamp = navigator.get_clock().now().to_msg()
    a_pose.pose.position.x = 0.22
    #这是调的
    a_pose.pose.position.y = -3.1
    
    
    #a_pose.pose.position.y = -3.2
    
    # a_pose.pose.orientation.z = -0.08
    # a_pose.pose.orientation.w = 0.98
    
    a_pose.pose.orientation.z = -0.7071
    a_pose.pose.orientation.w = 0.7071

    b_pose = PoseStamped()
    b_pose.header.frame_id = 'map'
    b_pose.header.stamp = navigator.get_clock().now().to_msg()
    b_pose.pose.position.x = 2.7
    b_pose.pose.position.y = -3.2
    b_pose.pose.orientation.z = 0.7
    b_pose.pose.orientation.w = 0.79

    # 从S点到A点
    
    if navigator.navigate_to_pose(a_pose):
            print("成功到达A点")
            #发布grasp_command = "grasp"，抓取物体并等待torelease
            #一直发送这个命令，直到grasp_state = "torelease"
            
            while True:
                rclpy.spin_once(navigator,timeout_sec=0.1)
                grasp_msg = String()
                grasp_msg.data = "grasp"
                navigator.grasp_command_publisher.publish(grasp_msg)
                print("好好好")
                print(navigator.grasp_state)
                if navigator.grasp_state == "torelease":
                    break        

            
            # grasp_msg = String()
            # grasp_msg.data = "grasp"
            # self.grasp_command_publisher.publish(grasp_msg)
    else:
            print("导航到A点失败")

    # 从A点到B点
    
    # while True:
    #     rclpy.spin_once(navigator)
    #     if self.grasp_state == "torelease":
    #         break 
    
    
    if navigator.navigate_to_pose(b_pose):
        #发布grasp_command = "release"，放下物体并等待finish
        while True:
            rclpy.spin_once(navigator,timeout_sec=0.1)
            
            grasp_msg = String()
            grasp_msg.data = "release"
            print("好好好hahhahaha,在B点")
            navigator.grasp_command_publisher.publish(grasp_msg)
            print("release published!")
            
            if navigator.grasp_state == "finish":
                break
        print("导航到B点并成功放下物体")

    
    else:
        print("导航到B点失败")

    
    # while True:
    #     rclpy.spin_once(navigator)
    #     if navigator.grasp_state == "finish":
    #         break
    #从B点返回S点
    if  navigator.navigate_to_pose(initial_pose):
        print("成功返回起点")
    else:
        print("返回起点失败")

    rclpy.shutdown()
    navigator.main()
    
