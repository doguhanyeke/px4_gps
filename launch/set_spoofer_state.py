#!/usr/bin/env python3
import numpy as np
from numpy.core.arrayprint import printoptions
from numpy.linalg import pinv

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from std_msgs.msg import Bool


from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SpooferTraj(Node):
    """
    Move a target entity(gazebo model) along a set trajectory defined by traj_f
    traj_f should always take @t and @begin_pose as the first two arguments
    """
    def __init__(self, target_name="spoofer") -> None:
        self.target_name = target_name
        super().__init__('spoofer_gz_stream')
        ## Configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        #PX4_NS = os.getenv("PX4_MICRODDS_NS")
        PX4_NS = "px4_1"
        fmu = f"{PX4_NS}/fmu"
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            f"{fmu}/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{fmu}/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile)
        self.spoofing_flag_pub = self.create_publisher(Bool, "/spoofing_flag", 10)
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])

        self.client = self.create_client(SetEntityPose, "/world/AbuDhabi/set_pose")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gazebo set_entity_state service is not available, waiting...')

        print(f"setting state for {self.target_name}")
        self.entity = Entity()
        self.entity.name = self.target_name
        self.request = SetEntityPose.Request()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.count = 0 
        self.mode = 0 
    
    def vector2PoseMsg(self,position, attitude):
        pose_msg = Pose()
        pose_msg.orientation.w = attitude[0]
        pose_msg.orientation.x = attitude[1]
        pose_msg.orientation.y = attitude[2]
        pose_msg.orientation.z = attitude[3]
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        return pose_msg
    
    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.y
        self.vehicle_local_position[1] = msg.x
        self.vehicle_local_position[2] = -msg.z + 10
        self.vehicle_local_velocity[0] = msg.vy
        self.vehicle_local_velocity[1] = msg.vx
        self.vehicle_local_velocity[2] = -msg.vz

    def cmdloop_callback(self):
        bool_msg = Bool()
        bool_msg.data = True
        self.spoofing_flag_pub.publish(bool_msg)
        
        spoofer_position = [self.count*0.02, self.count*0.02, 5.0] 
        
        if self.mode ==0:
            self.count+=1
            if (self.count>100):
                self.mode=1
        else:
            self.count-=1
            if (self.count<0):
                self.count=0 
                    
        vehicle_pose_msg = self.vector2PoseMsg(spoofer_position, self.vehicle_attitude)
        self.request.entity = self.entity
        self.request.pose = vehicle_pose_msg
        future = self.client.call_async(self.request)
        if future.done():
            response = future.result()
            print("response: " + response)

def main(args=None):
    rclpy.init(args=args)
    
    # executor = rclpy.get_global_executor()
    executor = MultiThreadedExecutor()
    traj_1 = SpooferTraj()
    executor.add_node(traj_1)
    executor.spin()

    try:
        rclpy.shutdown()
    except Exception():
        pass

if __name__ == '__main__':
    main()