#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import rclpy
import tf_transformations

from rclpy.node import Node
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from robot_interfaces.srv import MarkStopPoint


class PathRecorderNode(Node):
    """Record the robot path from TF and save it as a waypoint CSV."""

    def __init__(self):
        """Initialize parameters, TF listener, services, and recording timer."""
        super().__init__('path_recorder_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_waypoint_spacing_m', 0.2)
        self.declare_parameter('record_rate_hz', 10.0)
        self.declare_parameter('output_path', '/home/robot/dev/Pashupati/map')

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._min_spacing = float(self.get_parameter('min_waypoint_spacing_m').value)
        record_rate = float(self.get_parameter('record_rate_hz').value)
        self._output_path = self.get_parameter('output_path').value

        self._recording = False
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._recording_file = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_service(SetBool, 'mapping/path_record', self.recording_callback)
        self.create_service(MarkStopPoint, 'mapping/mark_stop_point', self.mark_stop_point_callback)
        self.create_timer(1.0 / record_rate, self.record_tick)

    def recording_callback(self, request, response):
        """Start or stop path recording."""
        if request.data:
            if self._recording:
                response.success = False
                response.message = 'already recording'
                return response

            self._waypoints = []
            self._recording = True

            timestamp = datetime.now()
            date_dir = timestamp.strftime('%Y-%m-%d')
            time_name = timestamp.strftime('%H-%M-%S')
            record_dir = os.path.join(self._output_path, date_dir)

            os.makedirs(record_dir, exist_ok=True)
            self._recording_file = os.path.join(record_dir, f'{time_name}_path.csv')

            response.success = True
            response.message = 'recording started'
            self.get_logger().info(f'path recording started: {self._recording_file}')
            return response

        if not self._recording:
            response.success = False
            response.message = 'not recording'
            return response

        self._recording = False
        self.write_csv()

        response.success = True
        response.message = f'recording stopped, {len(self._waypoints)} waypoints saved to {self._recording_file}'
        self.get_logger().info(response.message)
        return response

    def lookup_current_pose(self):
        """Look up the current map->base_link pose."""
        try:
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception as error:
            self.get_logger().warning(f'TF lookup {self._map_frame}->{self._base_frame} failed: {error}', throttle_duration_sec=2.0)
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        return x, y, math.degrees(yaw)

    def record_tick(self):
        """Sample and record the robot pose when it moved far enough."""
        if not self._recording:
            return

        pose = self.lookup_current_pose()
        if pose is None:
            return

        x, y, yaw_deg = pose

        if self._waypoints:
            last_x, last_y, _, _ = self._waypoints[-1]
            if math.hypot(x - last_x, y - last_y) < self._min_spacing:
                return

        self._waypoints.append((x, y, yaw_deg, 0.0))

    def mark_stop_point_callback(self, request, response):
        """Append the current pose as an inspection stop point."""
        if not self._recording:
            response.success = False
            response.message = 'not recording, call mapping/path_record(true) first'
            return response

        pose = self.lookup_current_pose()
        if pose is None:
            response.success = False
            response.message = 'TF not available, could not sample current pose'
            return response

        x, y, yaw_deg = pose
        self._waypoints.append((x, y, yaw_deg, request.dwell_sec))

        response.success = True
        response.message = f'stop point marked at ({x:.2f}, {y:.2f}), dwell {request.dwell_sec:.1f}s'
        self.get_logger().info(response.message)
        return response

    def write_csv(self):
        """Write recorded waypoints to the current recording CSV file."""
        if not self._recording_file:
            self.get_logger().error('No recording file configured.')
            return

        os.makedirs(os.path.dirname(self._recording_file), exist_ok=True)

        with open(self._recording_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['# x', 'y', 'yaw_deg', 'dwell_sec'])

            for x, y, yaw_deg, dwell_sec in self._waypoints:
                writer.writerow([f'{x:.4f}', f'{y:.4f}', f'{yaw_deg:.2f}', f'{dwell_sec:.1f}'])


def main(args=None):
    """Initialize ROS 2 and run the path recorder node."""
    rclpy.init(args=args)
    node = PathRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()