#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler


WORKSPACE_SRC = '/workspace/ros2_ws/src/turtlebot3_llm_nav2'
SCRIPTS_DIR = os.path.join(WORKSPACE_SRC, 'scripts')
PACKAGE_ROOT = WORKSPACE_SRC

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    from call_llm import call_llm
except Exception:
    call_llm = None


class ExperimentNavigator(Node):
    MODES = {
        'nav2_only',
        'llm_oneshot',
        'llm_periodic',
        'llm_failure_aware',
    }

    def __init__(self, args):
        super().__init__('experiment_navigator_metrics')
        self.args = args
        self.mode = args.mode
        self.goal_xy = self._resolve_goal(args.goal)
        self.start_time = time.time()
        self.last_feedback_time = None
        self.last_progress_time = time.time()
        self.best_distance_remaining = None
        self.prev_poses_remaining = None
        self.prev_recoveries = 0
        self.recovery_events = []
        self.current_pose_map = None
        self.current_pose_odom = None
        self.current_speed = 0.0
        self.last_path_pose_map = None
        self.last_path_pose_odom = None
        self.path_length_map_m = 0.0
        self.path_length_odom_m = 0.0
        self.goal_handle = None
        self.goal_done = False
        self.replan_in_progress = False
        self.last_replan_time = 0.0
        self.pending_replan_reason = None
        self.canceling_for_replan = False

        self.metrics = {
            'environment': args.environment or args.scenario,
            'scenario': args.scenario,
            'mode': self.mode,
            'method': self._method_name(self.mode),
            'trial_id': args.trial_id,
            'start': '',
            'goal': json.dumps(self.goal_xy),
            'success': False,
            'status': 'not_started',
            'path_length_m': 0.0,
            'navigation_time_s': 0.0,
            'llm_inference_time_s': 0.0,
            'num_llm_calls': 0,
            'num_replans': 0,
            'num_nav2_failures': 0,
            'num_recoveries': 0,
            'num_stuck_triggers': 0,
            'path_length_source': 'map',
            'path_length_odom_m': 0.0,
            'final_distance_remaining': '',
            'failure_reason': '',
        }

        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)

        if self.mode == 'llm_periodic':
            self.create_timer(args.period_sec, self._periodic_replan_timer)

        self.create_timer(args.monitor_period_sec, self._monitor_timer)
        self.create_timer(0.5, self._start_once_timer)
        self.started = False

    def _start_once_timer(self):
        if self.started:
            return
        self.started = True

        if self.mode == 'nav2_only':
            waypoints = [self.goal_xy]
        elif self.mode in {'llm_oneshot', 'llm_periodic', 'llm_failure_aware'}:
            if self.args.replan_on_start:
                waypoints = self._run_llm_replan('oneshot_start')
            else:
                waypoints = self._load_waypoints()
                self.metrics['num_llm_calls'] = 1
                if self.args.initial_llm_inference_time_sec is not None:
                    self.metrics['llm_inference_time_s'] = round(
                        float(self.args.initial_llm_inference_time_sec), 3
                    )
        else:
            raise RuntimeError(f'Unsupported mode: {self.mode}')

        if self.args.start is not None:
            self.metrics['start'] = json.dumps([float(self.args.start[0]), float(self.args.start[1])])
        elif waypoints:
            self.metrics['start'] = json.dumps([float(waypoints[0][0]), float(waypoints[0][1])])

        self._send_waypoints(waypoints)

    def _resolve_goal(self, goal_arg):
        if goal_arg is not None:
            return [float(goal_arg[0]), float(goal_arg[1])]

        try:
            waypoints = self._load_waypoints(log=False)
            if waypoints:
                return [float(waypoints[-1][0]), float(waypoints[-1][1])]
        except Exception:
            pass

        raise RuntimeError('Goal is required. Use --goal X Y or prepare waypoints.json first.')

    def _load_waypoints(self, log=True):
        with open(self.args.waypoints_file) as f:
            waypoints = json.load(f)['waypoints']
        normalized = [[float(x), float(y)] for x, y in waypoints]
        if log:
            self.get_logger().info(f'Loaded {len(normalized)} waypoints from {self.args.waypoints_file}')
        return normalized

    def _current_start(self):
        pose = self.current_pose_map or self.current_pose_odom
        if pose is None:
            if self.args.start is not None:
                return [float(self.args.start[0]), float(self.args.start[1])]
            self.get_logger().warn('Current pose is unknown. Falling back to [0.0, 0.0].')
            return [0.0, 0.0]
        return [float(pose[0]), float(pose[1])]

    def _run_llm_replan(self, reason):
        if call_llm is None:
            raise RuntimeError('call_llm.py could not be imported.')

        start = self._current_start()
        goal = self.goal_xy
        self.get_logger().info(f'Calling LLM for waypoints. reason={reason}, start={start}, goal={goal}')
        self.metrics['num_llm_calls'] += 1
        llm_start = time.time()
        call_llm(start, goal, self.args.obstacle_file, self.args.waypoints_file)
        elapsed = time.time() - llm_start
        self.metrics['llm_inference_time_s'] = round(
            float(self.metrics['llm_inference_time_s']) + elapsed, 3
        )
        return self._load_waypoints()

    def _send_waypoints(self, waypoints):
        if not waypoints:
            raise RuntimeError('No waypoints available.')

        poses = []
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            if i < len(waypoints) - 1:
                nx, ny = waypoints[i + 1]
                yaw = math.atan2(float(ny) - float(y), float(nx) - float(x))
            else:
                yaw = self._yaw_to_goal(x, y)

            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = ''

        self.get_logger().info(f'Sending {len(poses)} poses to NavigateThroughPoses. mode={self.mode}')
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        self.goal_done = False
        self.metrics['status'] = 'running'

    def _yaw_to_goal(self, x, y):
        dx = float(self.goal_xy[0]) - float(x)
        dy = float(self.goal_xy[1]) - float(y)
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        return math.atan2(dy, dx)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            self.metrics['num_nav2_failures'] += 1
            self.metrics['failure_reason'] = 'goal_rejected'
            if self.mode == 'llm_failure_aware':
                self._request_replan('goal_rejected')
            else:
                self._finish(False, 'goal_rejected')
            return

        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result_msg = future.result()
        status = result_msg.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation finished successfully')
            self._finish(True, 'succeeded')
            return

        status_name = self._status_name(status)
        if status == GoalStatus.STATUS_CANCELED and self.canceling_for_replan:
            self.get_logger().info('Previous goal canceled for replanning.')
            self.canceling_for_replan = False
            return

        self.get_logger().warn(f'Navigation ended with status={status_name}')
        self.metrics['num_nav2_failures'] += 1

        if self.mode == 'llm_failure_aware' and not self.goal_done:
            self._request_replan(f'nav2_{status_name.lower()}')
        else:
            self._finish(False, status_name.lower())

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        now = time.time()
        self.last_feedback_time = now
        self.metrics['final_distance_remaining'] = round(float(feedback.distance_remaining), 3)

        if self.prev_poses_remaining is None or feedback.number_of_poses_remaining < self.prev_poses_remaining:
            self.get_logger().info(
                f'Waypoint reached. remaining={feedback.number_of_poses_remaining}, '
                f'recoveries={feedback.number_of_recoveries}, '
                f'distance={feedback.distance_remaining:.3f}'
            )
        self.prev_poses_remaining = feedback.number_of_poses_remaining

        if feedback.number_of_recoveries > self.prev_recoveries:
            self.recovery_events.append(now)
            self.metrics['num_recoveries'] = int(feedback.number_of_recoveries)
            self.get_logger().warn(f'Nav2 recovery count increased: {feedback.number_of_recoveries}')
        self.prev_recoveries = int(feedback.number_of_recoveries)

        distance = float(feedback.distance_remaining)
        if self.best_distance_remaining is None:
            self.best_distance_remaining = distance
            self.last_progress_time = now
        elif distance < self.best_distance_remaining - self.args.progress_epsilon:
            self.best_distance_remaining = distance
            self.last_progress_time = now

        if self.mode == 'llm_failure_aware':
            self._check_failure_aware_triggers(now)

    def _check_failure_aware_triggers(self, now):
        if self.replan_in_progress or self.goal_done:
            return
        if now - self.last_replan_time < self.args.replan_cooldown_sec:
            return

        no_progress_sec = now - self.last_progress_time
        low_speed = self.current_speed < self.args.low_speed_threshold

        if low_speed and no_progress_sec >= self.args.stuck_time_sec:
            self.metrics['num_stuck_triggers'] += 1
            self._request_replan('stuck_low_speed_no_progress')
            return

        recent_recoveries = [t for t in self.recovery_events if now - t <= self.args.recovery_window_sec]
        if len(recent_recoveries) >= self.args.recovery_trigger_count:
            self._request_replan('repeated_recovery')

    def _request_replan(self, reason):
        if self.replan_in_progress or self.goal_done:
            return
        self.replan_in_progress = True
        self.pending_replan_reason = reason
        self.last_replan_time = time.time()
        self.metrics['num_replans'] += 1
        self.get_logger().warn(f'Replan requested: {reason}')

        if self.goal_handle is not None:
            self.canceling_for_replan = True
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
        else:
            self._perform_replan()

    def _cancel_done_callback(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn(f'Goal cancel failed before replanning: {exc}')
        self._perform_replan()

    def _perform_replan(self):
        try:
            waypoints = self._run_llm_replan(self.pending_replan_reason or 'replan')
            self._reset_progress_tracking()
            self._send_waypoints(waypoints)
        except Exception as exc:
            self.get_logger().error(f'Replanning failed: {exc}')
            self._finish(False, 'replanning_failed')
        finally:
            self.replan_in_progress = False
            self.pending_replan_reason = None

    def _periodic_replan_timer(self):
        if not self.started or self.goal_done or self.replan_in_progress:
            return
        if time.time() - self.last_replan_time < self.args.replan_cooldown_sec:
            return
        self._request_replan('periodic')

    def _monitor_timer(self):
        if not self.started or self.goal_done:
            return
        if self.last_feedback_time is None:
            return

        feedback_age = time.time() - self.last_feedback_time
        if feedback_age > self.args.action_feedback_timeout_sec:
            self.get_logger().warn(f'No Nav2 feedback for {feedback_age:.1f} sec')
            if self.mode == 'llm_failure_aware':
                self._request_replan('feedback_timeout')

    def _reset_progress_tracking(self):
        self.last_progress_time = time.time()
        self.best_distance_remaining = None
        self.prev_poses_remaining = None
        self.prev_recoveries = 0
        self.recovery_events = []

    def _amcl_callback(self, msg):
        pose = msg.pose.pose
        yaw = self._yaw_from_pose(pose)
        self.current_pose_map = [pose.position.x, pose.position.y, yaw]
        self._accumulate_path_length('map', pose.position.x, pose.position.y)

    def _odom_callback(self, msg):
        pose = msg.pose.pose
        yaw = self._yaw_from_pose(pose)
        self.current_pose_odom = [pose.position.x, pose.position.y, yaw]
        self._accumulate_path_length('odom', pose.position.x, pose.position.y)
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        self.current_speed = math.sqrt(linear.x ** 2 + linear.y ** 2) + abs(angular.z) * 0.1

    def _accumulate_path_length(self, source, x, y):
        if not self.started or self.goal_done:
            return

        current = [float(x), float(y)]
        if source == 'map':
            previous = self.last_path_pose_map
            self.last_path_pose_map = current
            if previous is None:
                return
            distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
            if distance <= self.args.max_pose_jump_m:
                self.path_length_map_m += distance
            return

        previous = self.last_path_pose_odom
        self.last_path_pose_odom = current
        if previous is None:
            return
        distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
        if distance <= self.args.max_pose_jump_m:
            self.path_length_odom_m += distance

    def _yaw_from_pose(self, pose):
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def _finish(self, success, status):
        if self.goal_done:
            return
        self.goal_done = True
        self.metrics['success'] = bool(success)
        self.metrics['status'] = status
        self.metrics['navigation_time_s'] = round(time.time() - self.start_time, 3)
        self.metrics['path_length_odom_m'] = round(self.path_length_odom_m, 3)
        if self.path_length_map_m > 0.0:
            self.metrics['path_length_m'] = round(self.path_length_map_m, 3)
            self.metrics['path_length_source'] = 'map'
        else:
            self.metrics['path_length_m'] = round(self.path_length_odom_m, 3)
            self.metrics['path_length_source'] = 'odom'
        self._write_metrics()
        rclpy.shutdown()

    def _write_metrics(self):
        output_path = self.args.results_file
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        row = dict(self.metrics)
        row['timestamp'] = datetime.now().isoformat(timespec='seconds')

        fieldnames = [
            'timestamp',
            'environment',
            'scenario',
            'mode',
            'method',
            'trial_id',
            'start',
            'goal',
            'success',
            'status',
            'path_length_m',
            'navigation_time_s',
            'llm_inference_time_s',
            'num_llm_calls',
            'num_replans',
            'num_nav2_failures',
            'num_recoveries',
            'num_stuck_triggers',
            'path_length_source',
            'path_length_odom_m',
            'final_distance_remaining',
            'failure_reason',
        ]
        file_exists = os.path.exists(output_path)
        with open(output_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)
        self.get_logger().info(f'Wrote metrics to {output_path}')

    def _status_name(self, status):
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'STATUS_{status}')

    def _method_name(self, mode):
        names = {
            'nav2_only': 'Nav2/DWA only',
            'llm_oneshot': 'Original LLM-DWA',
            'llm_periodic': 'Periodic LLM-DWA replanning',
            'llm_failure_aware': 'Failure-aware LLM-DWA replanning',
        }
        return names.get(mode, mode)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Experimental runner for Nav2/DWA and LLM-DWA replanning baselines.'
    )
    parser.add_argument(
        '--mode',
        required=True,
        choices=sorted(ExperimentNavigator.MODES),
        help='Experiment mode to run.',
    )
    parser.add_argument('--scenario', default='unknown')
    parser.add_argument('--environment', default=None)
    parser.add_argument('--trial-id', default='0')
    parser.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument(
        '--obstacle-file',
        default='/workspace/ros2_ws/src/turtlebot3_llm_nav2/data/obstacle.json',
    )
    parser.add_argument(
        '--waypoints-file',
        default='/workspace/ros2_ws/src/turtlebot3_llm_nav2/data/waypoints.json',
    )
    parser.add_argument(
        '--results-file',
        default='/workspace/ros2_ws/src/turtlebot3_llm_nav2/results/experiment_results.csv',
    )
    parser.add_argument('--period-sec', type=float, default=15.0)
    parser.add_argument('--stuck-time-sec', type=float, default=10.0)
    parser.add_argument('--progress-epsilon', type=float, default=0.2)
    parser.add_argument('--low-speed-threshold', type=float, default=0.03)
    parser.add_argument('--recovery-trigger-count', type=int, default=2)
    parser.add_argument('--recovery-window-sec', type=float, default=20.0)
    parser.add_argument('--replan-cooldown-sec', type=float, default=8.0)
    parser.add_argument('--monitor-period-sec', type=float, default=1.0)
    parser.add_argument('--action-feedback-timeout-sec', type=float, default=15.0)
    parser.add_argument('--max-pose-jump-m', type=float, default=1.0)
    parser.add_argument(
        '--initial-llm-inference-time-sec',
        type=float,
        default=None,
        help='Optional measured time from the initial run_llm_pipeline.py call.',
    )
    parser.add_argument(
        '--replan-on-start',
        action='store_true',
        help='Call the same LLM pipeline once at startup instead of reading waypoints.json.',
    )
    parsed, _ = parser.parse_known_args(argv)
    return parsed


def main(args=None):
    parsed = parse_args(args)
    rclpy.init(args=args)
    node = ExperimentNavigator(parsed)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
