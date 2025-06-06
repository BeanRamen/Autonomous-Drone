#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/gazebo_ros_head_rplidar_controller/out', self.lidar_callback, qos_profile)
        self.create_subscription(
            String, '/person_position', self.person_callback, qos_profile)
        self.create_subscription(
            Float32MultiArray, '/clicked_waypoints', self.waypoints_callback, qos_profile)

        
        self.person_seen = False
        self.last_position = None   
        self.obstacle_detected = False
        self.obstacle_distance_threshold = 5.0 
        self.vehicle_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.current_waypoint_index = 0
        #self.waypoints = [[0.0, 0.0, -5.0], [50.0, 10.0, -5.0], [105.0, -1.23, -5.0], [-266,-34,-5], [0.0, 0.0, -5.0]]
        self.waypoints = [[0.0, 0.0, -5.0]]
        self.yaw = 0.0
        self.waypoint_tolerance = 2.0 

        self.blocked_counter = 0 
        self.blocked_threshold = 30

        self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter = 0

    def waypoints_callback(self, msg):
        self.waypoints = []
        for i in range(0, len(msg.data), 3):
            x = msg.data[i]
            y = msg.data[i + 1]
            z = msg.data[i + 2]
            self.waypoints.append([x, y, z])
        self.get_logger().info(f"Waypoints: {self.waypoints}")
        self.current_waypoint_index = 0
    
    # def person_callback(self, msg):
    #     data = msg.data
    #     if data.startswith("person:"):
    #         self.person_seen = True
    #         direction = data.split(":")[1]
    #         x = self.vehicle_position.x
    #         y = self.vehicle_position.y
    #         z = self.vehicle_position.z
    #         if direction == "left":
    #             self.yaw -= 0.2
    #             self.publish_position_setpoint(x, y, z)
    #         elif direction == "right":
    #             self.yaw += 0.2
    #             self.publish_position_setpoint(x, y, z)
    #         elif direction == "center":
    #             if x >= 0:
    #                 new_x = x + 0.1
    #             else:
    #                 new_x = x - 0.1
    #             self.publish_position_setpoint(new_x, y, z)
    #     else : self.person_seen = False
    
    def person_callback(self, msg):
        data = msg.data
        if not data.startswith("person:"):
            self.person_seen = False
            return
        
        self.person_seen = True
        direction = data.split(":",1)[1]
        x = self.vehicle_position.x
        y = self.vehicle_position.y
        z = self.vehicle_position.z 

        if direction == "left":
            self.yaw -= 0.2
        elif direction == "right":
            self.yaw += 0.2
        self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi

        if direction in ("left", "right"):
            self.publish_position_setpoint(x, y, z)
        else:
            new_x = x + 0.2 * math.cos(self.yaw)
            new_y = y + 0.2 * math.sin(self.yaw)
            self.publish_position_setpoint(new_x, new_y, z)
    
    def lidar_callback(self, msg):
        x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z

        wx, wy, wz = self.waypoints[self.current_waypoint_index]
        
        threshold = self.obstacle_distance_threshold
        ranges = [r for r in msg.ranges if r > 2]
        total = len(ranges)
        right = ranges[:total // 4]
        center = ranges[total // 4 : 3 * total // 4]
        left = ranges[3 * total // 4:]

        valid_right = [r for r in right if not math.isinf(r) and r > 2]
        valid_center = [r for r in center if not math.isinf(r) and r > 2]
        valid_left = [r for r in left if not math.isinf(r) and r > 2]

        min_distance_right = min(valid_right) if valid_right else float('inf')
        min_distance_center = min(valid_center) if valid_center else float('inf')
        min_distance_left = min(valid_left) if valid_left else float('inf')

        max_distance_right = max(valid_right) if valid_right else float('inf')
        max_distance_center = max(valid_center) if valid_center else float('inf')
        max_distance_left = max(valid_left) if valid_left else float('inf')

        min_distances = [min_distance_left, min_distance_center, min_distance_right]
        max_distances = [max_distance_left, max_distance_center, max_distance_right]

        if not any(2< r < threshold and not math.isinf(r) for r in msg.ranges):
            self.obstacle_detected = False
            return
            
        if min_distance_center < self.obstacle_distance_threshold:
            self.obstacle_detected = True
            self.avoid_obstacle(min_distances,max_distances)
        else:
            self.obstacle_detected = False


    def avoid_obstacle(self,min_distances,max_distances):
        min_left, min_center, min_right = min_distances
        max_left, max_center, max_right = max_distances
        x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z
        wx, wy, wz = self.waypoints[self.current_waypoint_index]

        side_step = 0.5
        if min_left > min_right and min_left > self.obstacle_distance_threshold:
            new_x = x + side_step * math.sin(self.yaw)
            new_y = y - side_step * math.cos(self.yaw)
            new_z = wz
            #self.get_logger().info(f"Ocolesc pe STANGA {min_right:.2f}")
        elif min_right > min_left and min_right > self.obstacle_distance_threshold:
            new_x = x - side_step * math.sin(self.yaw)
            new_y = y + side_step * math.cos(self.yaw)
            new_z = wz
            #self.get_logger().info(f"Ocolesc pe DREAPTA {min_left:.2f}")
        # elif (max_left == max_right):
        #     new_x = x
        #     if min_left >= min_right:
        #         new_y = y + side_step * math.sin(self.yaw)
        #     else:
        #         new_y = y - side_step * math.sin(self.yaw)

        #     new_z = wz
        #     self.yaw = self.yaw
        #     #self.get_logger().info(f"Ocolesc pe STANGA {min_right:.2f}")
        else:
            new_x = x 
            new_y = y
            self.waypoints[self.current_waypoint_index][2] -= 0.5
            new_z = self.waypoints[self.current_waypoint_index][2]
            #self.get_logger().warn(f"NU EXISTĂ CALE LIBERĂ! Aștept...[{min_left:.2f} / {min_right:.2f}]")

        self.yaw = math.atan2(wy - new_y, wx - new_x)
        self.publish_position_setpoint(new_x, new_y, new_z)

    def vehicle_local_position_callback(self, position):
        self.vehicle_position = position
        wx, wy, wz = self.waypoints[self.current_waypoint_index]
        x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z
        if self.person_seen == False:
            self.get_logger().info(f"POS X [{x:.2f}m]  Y [{y:.2f}m]  Z [{z:.2f}m]")
            a = 2 
            self.get_logger().info(f"POS WX [{wx:.2f}m]  WY [{wy:.2f}m]  WZ [{wz:.2f}m] YAW [{self.yaw:.2f}]")
        else:
            self.get_logger().info(f"POS X [{x:.2f}m]  Y [{y:.2f}m]  Z [{z:.2f}m YAW [{self.yaw:.2f}] Persoana detectata! Astept...")
            
    def vehicle_status_callback(self, status):
        self.vehicle_status = status

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def return_to_home(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def publish_offboard_control_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def hold(self, x: float, y: float, z: float):
        wx, wy, wz = self.waypoints[self.current_waypoint_index]
        curr = self.vehicle_position
        dx = x - curr.x
        dy = y - curr.y
        dz = z - curr.z

        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist < self.waypoint_tolerance:
            vx = vy = vz = 0.0
        else:
            speed = 3.0 
            zspeed = 3.0
            vx = speed * dx / dist
            vy = speed * dy / dist
            vz = zspeed * dz / dist

        msg = TrajectorySetpoint()
        msg.position = [float(wx), float(wz), float(wz)]
        msg.velocity = [vx, vy, vz]
        msg.yaw = self.yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        wx, wy, wz = self.waypoints[self.current_waypoint_index]
        curr = self.vehicle_position
        dx = x - curr.x
        dy = y - curr.y
        dz = z - curr.z

        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist < 0.1:
            vx = vy = vz = 0.0
        else:
            speed = 3.0 
            zspeed = 3.0
            vx = speed * dx / dist
            vy = speed * dy / dist
            vz = zspeed * dz / dist

        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, float(wz)]
        msg.velocity = [vx, vy, vz]
        msg.yaw = self.yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def distance_to_waypoint(self, current_pos, waypoint):
        return ((current_pos.x - waypoint[0]) ** 2 + (current_pos.y - waypoint[1]) ** 2 + (current_pos.z - waypoint[2]) ** 2) ** 0.5

    def timer_callback(self):
        self.publish_offboard_control_heartbeat()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z
        if self.last_position is not None:
            last_x, last_y, last_z = self.last_position
            tolerance = 0.2
            if abs(x - last_x) <= tolerance and abs(y - last_y) <= tolerance:
                self.blocked_counter += 1
                if self.blocked_counter >= self.blocked_threshold:
                    #self.get_logger().warn("Vehicul Blocat: Nu se mișcaa!")
                    self.publish_offboard_control_heartbeat()
                    if self.offboard_setpoint_counter == 10:
                        self.engage_offboard_mode()
                        self.arm()
                    self.publish_position_setpoint(x, y, z - 20.0)
                    self.blocked_counter = 0
            else:
                self.blocked_counter = 0
        self.last_position = (x, y, z)

        if self.distance_to_waypoint(self.vehicle_position, self.waypoints[self.current_waypoint_index]) > self.waypoint_tolerance:
            if self.obstacle_detected == False and self.person_seen == False:
                wx, wy, wz = self.waypoints[self.current_waypoint_index]
                x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z
                self.yaw = math.atan2(wy - y, wx - x)
                self.publish_position_setpoint(wx, wy, wz)
        else:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.waypoints = [[0.0, 0.0,-5.0]]
                self.current_waypoint_index = 0

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


    

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

