#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan

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

        self.obstacle_detected = False
        self.obstacle_distance_threshold = 10.0 
        self.vehicle_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.current_waypoint_index = 0
        self.waypoints = [[0.0, 0.0, -5.0],[-266,-34,-10], [50.0, 10.0, -10.0], [105.0, -1.23, -10.0], [0.0, 0.0, -5.0]]
        self.yaw = 0.0
        self.waypoint_tolerance = 2.0  # Toleranța pentru a considera că drona a ajuns la un waypoint

        self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter = 0

    def lidar_callback(self, msg):
        x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z

        wx, wy, wz = self.waypoints[self.current_waypoint_index]
        
        threshold = self.obstacle_distance_threshold

        ranges = [r for r in msg.ranges if r > 2]
        treime = len(ranges) //3
        right = ranges[:treime]
        center = ranges[treime:2 * treime]
        left = ranges[2 * treime:]

        valid_right = [r for r in right if not math.isinf(r) and r > 2]
        valid_center = [r for r in center if not math.isinf(r) and r > 2]
        valid_left = [r for r in left if not math.isinf(r) and r > 2]

        min_distance_right = min(valid_right) if valid_right else float('inf')
        min_distance_center = min(valid_center) if valid_center else float('inf')
        min_distance_left = min(valid_left) if valid_left else float('inf')

        min_distances = [min_distance_left, min_distance_center, min_distance_right]

        if min_distance_right < threshold:
            obsright = "dreapta"
        if min_distance_center < threshold:
            obscenter = "mijloc"
        if min_distance_left < threshold:
            obsleft = "stanga"

        if not any(2< r < threshold and not math.isinf(r) for r in msg.ranges):
            self.get_logger().info(f"Obstacole indepartate la {min(min_distances):.2f}m")
            self.get_logger().info(f"POS X [{x:.2f}m]  Y [{y:.2f}m]  Z [{z:.2f}m]")
            self.get_logger().info(f"POS WX [{wx:.2f}m]  WY [{wy:.2f}m]  WZ [{wz:.2f}m]")
            self.obstacle_detected = False
            return
        
        self.get_logger().info(f"POS X [{x:.2f}m]  Y [{y:.2f}m]  Z [{z:.2f}m]")
        self.get_logger().info(f"POS WX [{wx:.2f}m]  WY [{wy:.2f}m]  WZ [{wz:.2f}m]")
            
        if min_distance_center < self.obstacle_distance_threshold or min(min_distances) < 5:
            self.obstacle_detected = True
            self.avoid_obstacle(min_distances)
        else:
            self.obstacle_detected = False


    def avoid_obstacle(self,min_distances):
        min_left, min_center, min_right = min_distances
        x, y, z = self.vehicle_position.x, self.vehicle_position.y, self.vehicle_position.z
        wx, wy, wz = self.waypoints[self.current_waypoint_index]

        self.get_logger().info(f"POS X [{x:.2f}m]  Y [{y:.2f}m]  Z [{z:.2f}m]")
        self.get_logger().info(f"POS WX [{wx:.2f}m]  WY [{wy:.2f}m]  WZ [{wz:.2f}m]")
        side_step = 0.5
        if min_left > min_right and min_left > self.obstacle_distance_threshold:
            
            new_x = x
            if y > 0:
                new_y = y - side_step
            else:
                new_y = y + side_step
            new_z = z
            self.get_logger().info(f"Ocolesc pe STANGA {min_right:.2f}")
        elif min_right > min_left and min_right > self.obstacle_distance_threshold:
            new_x = x
            if y > 0:
                new_y = y + side_step
            else:
                new_y = y - side_step
            new_z = z
            self.get_logger().info(f"Ocolesc pe DREAPTA {min_left:.2f}")
        else:
            new_x = x
            new_y = y
            new_z = z - 10
            self.get_logger().warn("NU EXISTĂ CALE LIBERĂ! Aștept...")

        self.publish_position_setpoint(new_x, new_y, new_z)

    def vehicle_local_position_callback(self, position):
        """Callback pentru poziția locală a dronei."""
        self.vehicle_position = position
        #self.get_logger().info(f"Current pos: {self.vehicle_position.x:.2f}, {self.vehicle_position.y:.2f}, {self.vehicle_position.z:.2f}")


    def vehicle_status_callback(self, status):
        """Callback pentru starea dronei."""
        self.vehicle_status = status

    def arm(self):
        """Armare drona"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        #self.get_logger().info('Arm command sent')

    def disarm(self):
        """Dezarmare drona"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        #self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Activare mod offboard"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        #self.get_logger().info("Switching to offboard mode")

    def return_to_home(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def land(self):
        """Activare mod de aterizare"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        #self.get_logger().info("Landing initiated")

    def publish_offboard_control_heartbeat(self):
        """Trimite semnalul pentru a menține modul offboard activ"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):

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
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [vx, vy, vz]
        msg.yaw = self.yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Trimite o comandă către dronă"""
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
        """Calculează distanța până la un waypoint"""
        return ((current_pos.x - waypoint[0]) ** 2 + (current_pos.y - waypoint[1]) ** 2 + (current_pos.z - waypoint[2]) ** 2) ** 0.5

    def timer_callback(self):
        """Callback pentru timer"""
        self.publish_offboard_control_heartbeat()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.distance_to_waypoint(self.vehicle_position, self.waypoints[self.current_waypoint_index]) > self.waypoint_tolerance:
            if self.obstacle_detected == False:
                x, y, z = self.waypoints[self.current_waypoint_index]
                self.yaw = math.atan2(y,x)
                self.publish_position_setpoint(x, y, z)
        else:
            # Trecem la următorul waypoint
            self.current_waypoint_index += 1
            # Dacă am ajuns la ultimul waypoint, aterizăm
            if self.current_waypoint_index >= len(self.waypoints):
                self.return_to_home()
                self.land()

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

