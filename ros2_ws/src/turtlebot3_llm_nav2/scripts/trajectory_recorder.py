#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class TrajectoryRecorder(Node):
    def __init__(self, args):
        super().__init__('trajectory_recorder')
        self.args = args
        self.rows = []
        self.start_wall_time = time.time()
        self.last_write_time = 0.0

        if args.topic == '/amcl_pose':
            self.create_subscription(PoseWithCovarianceStamped, args.topic, self._amcl_callback, 20)
        elif args.topic == '/odom':
            self.create_subscription(Odometry, args.topic, self._odom_callback, 20)
        else:
            raise ValueError('Use --topic /amcl_pose or --topic /odom')

        self.get_logger().info(f'Recording {args.topic} to {args.out}')

    def _amcl_callback(self, msg):
        pose = msg.pose.pose
        self._record(pose.position.x, pose.position.y, pose.orientation)

    def _odom_callback(self, msg):
        pose = msg.pose.pose
        self._record(pose.position.x, pose.position.y, pose.orientation)

    def _record(self, x, y, orientation):
        now = time.time()
        if now - self.last_write_time < self.args.sample_period:
            return
        self.last_write_time = now
        _, _, yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        self.rows.append({
            't_sec': round(now - self.start_wall_time, 3),
            'x': round(float(x), 5),
            'y': round(float(y), 5),
            'yaw': round(float(yaw), 5),
        })

    def save(self):
        os.makedirs(os.path.dirname(self.args.out), exist_ok=True)
        with open(self.args.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['t_sec', 'x', 'y', 'yaw'])
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f'Saved {len(self.rows)} poses to {self.args.out}')


def parse_args():
    parser = argparse.ArgumentParser(description='Record robot trajectory from /amcl_pose or /odom.')
    parser.add_argument('--topic', default='/amcl_pose', choices=['/amcl_pose', '/odom'])
    parser.add_argument('--out', required=True)
    parser.add_argument('--sample-period', type=float, default=0.2)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TrajectoryRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
