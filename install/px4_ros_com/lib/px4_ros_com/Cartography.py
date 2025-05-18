#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import os

class Cartography(Node):
    def __init__(self):
        super().__init__('cartography_node')

        script_dir = os.path.dirname(os.path.realpath(__file__))
        map_path = os.path.abspath(os.path.join(script_dir, '/home/pavel/ws_ros2/src/px4_ros_com/src/examples/offboard_py/map/map.png'))

        self.map = cv2.imread(map_path)
        if self.map is None:
            self.get_logger().error(f"image not found {map_path}")
            rclpy.shutdown()
            return

        self.map_height, self.map_width = self.map.shape[:2]
        self.scale = 1.88
        self.origin_x = 451
        self.origin_y = 433

        self.overlay = self.map.copy()
        self.last_px = None
        self.cliecked_points = []
        self.default_z = -5.0

        self.waypoints_pub = self.create_publisher(Float32MultiArray, '/clicked_waypoints', 10)
        

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )
        self.timer = self.create_timer(0.1, self.display_map)
        cv2.namedWindow('Drone Cartography')
        cv2.setMouseCallback('Drone Cartography', self.mouse_callback)
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            world_y = (x - self.origin_x) / self.scale
            world_x = -(y - self.origin_y) / self.scale
            world_z = self.default_z
            self.cliecked_points.append((world_x, world_y, world_z))
            self.get_logger().info(f"Clicked point: {world_x}, {world_y}, {world_z}")
            cv2.circle(self.overlay, (x, y), 5, (255, 0, 0), -1)
            self.get_logger().info(f"Waypoints : {self.cliecked_points}")
        


    
        
    def position_callback(self, msg):
        x_ros = msg.y
        y_ros = msg.x

        px = int(self.origin_x + x_ros * self.scale)
        py = int(self.origin_y - y_ros * self.scale)

        if 0 <= px < self.map_width and 0 <= py < self.map_height:
            cv2.circle(self.overlay, (px, py), 2, (0, 0, 255), -1)
            if self.last_px is not None:
                cv2.line(self.overlay, self.last_px, (px, py), (0, 255, 0), 1)
            self.last_px = (px, py)

    def display_map(self):
        cv2.imshow('Drone Cartography', self.overlay)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info("Exit... Save map.")
            cv2.imwrite("traseu_final.png", self.overlay)
            rclpy.shutdown()
        elif key == 13:
            if not self.cliecked_points:
                self.get_logger().info("No points clicked.")
            else:
                msg = Float32MultiArray()
                flat = [coord for point in self.cliecked_points for coord in point]
                msg.data = flat
                self.waypoints_pub.publish(msg)
                self.get_logger().info(f"Publishing waypoints: {self.cliecked_points}")
                self.cliecked_points = []

def main(args=None):
    rclpy.init(args=args)
    node = Cartography()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
