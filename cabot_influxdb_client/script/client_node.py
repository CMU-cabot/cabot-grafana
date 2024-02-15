#!/usr/bin/env python3

# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math
import rclpy
from rclpy.node import Node
from cabot_msgs.msg import PoseLog, Log
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64

from cabot_ui import geoutil
from cabot_ui.cabot_rclpy_util import CaBotRclpyUtil

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime
import time
import sys


def throttle(interval_seconds):
    """
    A decorator to throttle function calls. The wrapped function can only be called
    once every interval_seconds. Subsequent calls within the interval are ignored.

    :param interval_seconds: The minimum time interval between function calls.
    :return: The wrapper function.
    """
    def decorator(func):
        last_called = [0]  # Use a mutable object to allow modification in nested scope
        def wrapper(*args, **kwargs):
            nonlocal last_called
            current_time = time.time()
            if current_time - last_called[0] >= interval_seconds:
                last_called[0] = current_time
                return func(*args, **kwargs)
        return wrapper
    return decorator


def get_nanosec(stamp=None):
    if stamp is None or \
       stamp.sec < 1e6:
        return int(time.time() * 1e9)
    return int(stamp.sec * 1e9 + stamp.nanosec)


class ClientNode(Node):
    def __init__(self):
        super().__init__("client_node")
        CaBotRclpyUtil.initialize(self)

        self.robot_name = self.declare_parameter("robot_name", "").value

        # to debug multiple robot visualization on grafana, just use this line
        # or set 'CABOT_INFLUXDB_ROBOT_NAME' to a comma separated names like
        # "cabot1,cabot2,cabot3"
        # self.robot_names = [f"cabot{i}" for i in range(1, 11)]
        self.robot_names = self.robot_name.split(",")
        self.host = self.declare_parameter("host", "").value
        self.token = self.declare_parameter("token", "").value
        self.org = self.declare_parameter("org", "").value
        self.bucket = self.declare_parameter("bucket", "").value
        anchor_file = self.declare_parameter("anchor_file", "").value
        image_left_topic = self.declare_parameter("image_left_topic", "").value
        image_center_topic = self.declare_parameter("image_center_topic", "").value
        image_right_topic = self.declare_parameter("image_right_topic", "").value
        self.get_logger().info(F"image_topics is {image_left_topic}, {image_center_topic}, {image_right_topic}")
        self.anchor = None
        self.get_logger().info(F"Anchor file is {anchor_file}")
        if anchor_file is not None:
            temp = geoutil.get_anchor(anchor_file)
            if temp is not None:
                self.anchor = temp
            else:
                self.get_logger.warn(F"could not load anchor_file \"{anchor_file}\"")
        pose_interval = self.declare_parameter("pose_interval", 1.0).value
        cmd_vel_interval = self.declare_parameter("cmd_vel_interval", 0.2).value
        odom_interval = self.declare_parameter("odom_interval", 0.2).value
        diag_agg_interval = self.declare_parameter("diag_agg_interval", 1.0).value
        image_interval = self.declare_parameter("image_interval", 5.0).value

        self.client = InfluxDBClient(url=self.host, token=self.token, org=self.org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        self.last_error = None

        self.pose_log_sub = self.create_subscription(PoseLog, '/cabot/pose_log', self.pose_log_callback(pose_interval), 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cabot/cmd_vel', self.cmd_vel_callback(cmd_vel_interval), 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback(odom_interval), 10)
        self.diag_agg_sub = self.create_subscription(DiagnosticArray, '/diagnostics_agg', self.diagnostics_callback(diag_agg_interval), 10)
        self.plan_sub = self.create_subscription(Path, '/path_all', self.path_callback, 10)
        if image_left_topic:
            self.image_left_sub = self.create_subscription(Image, image_left_topic, self.image_callback(image_interval, "left"), 10)
        if image_center_topic:
            self.image_center_sub = self.create_subscription(Image, image_center_topic, self.image_callback(image_interval, "center"), 10)
        if image_right_topic:
            self.image_right_sub = self.create_subscription(Image, image_right_topic, self.image_callback(image_interval, "right"), 10)
        self.event_sub = self.create_subscription(Log, '/cabot/activity_log', self.activity_log_callback, 10)
        self.bridge = CvBridge()

    def send_point(self, point):
        # prevent too much connection errors
        if self.last_error and time.time() - self.last_error < 1.0:
            return
        try :
            self.write_api.write(bucket=self.bucket, org=self.org, record=point)
        except Exception as e:
            self.get_logger().error(f"{e}")
            self.last_error = time.time()

    def pose_log_callback(self, interval):
        @throttle(interval)
        def inner_func(msg):
            orientation = msg.pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            count = 0
            for robot_name in self.robot_names:
                point = Point("pose_data") \
                    .field("lat", msg.lat + 0.0005 * count) \
                    .field("lng", msg.lng + 0.0005 * count) \
                    .field("floor", msg.floor) \
                    .field("yaw", - self.anchor.rotate - yaw/math.pi*180) \
                    .tag("robot_name", robot_name) \
                    .time(get_nanosec(), WritePrecision.NS)
                self.send_point(point)
                count += 1
        return inner_func


    def cmd_vel_callback(self, interval):
        @throttle(interval)
        def inner_func(msg):
            point = Point("cmd_vel") \
                .field("linear", msg.linear.x) \
                .field("angular", msg.angular.z) \
                .tag("robot_name", self.robot_name) \
                .time(get_nanosec(), WritePrecision.NS)
            self.send_point(point)
        return inner_func

    def odom_callback(self, interval):
        @throttle(interval)
        def inner_func(msg):
            for robot_name in self.robot_names:
                point = Point("odometry") \
                    .field("linear", msg.twist.twist.linear.x) \
                    .field("angular", msg.twist.twist.angular.z) \
                    .tag("robot_name", robot_name) \
                    .time(get_nanosec(msg.header.stamp), WritePrecision.NS)
                self.send_point(point)
        return inner_func

    def diagnostics_callback(self, interval):
        @throttle(interval)
        def inner_func(msg):
            diagnostics = {}
            for state in msg.status:
                items = state.name.split("/")
                if len(items) != 3:
                    continue
                name = items[2]
                if name not in diagnostics:
                    diagnostics[name] = 0

                level = int.from_bytes(state.level, byteorder='big')
                if diagnostics[name] < level:
                    diagnostics[name] = level

            for robot_name in self.robot_names:
                for name, level in diagnostics.items():
                    point = Point("diagnostic")\
                        .field("level", level)\
                        .tag("name", name)\
                        .tag("robot_name", robot_name) \
                        .time(get_nanosec(), WritePrecision.NS)
                    self.send_point(point)
        return inner_func

    def path_callback(self, msg):
        group = get_nanosec()
        count = 0
        for robot_name in self.robot_names:
            for pose in msg.poses:
                position = pose.pose.position
                localp = geoutil.Point(x=position.x, y=position.y)
                globalp = geoutil.local2global(localp, self.anchor)
                point = Point("plan") \
                    .field("lat", globalp.lat + 0.0005 * count) \
                    .field("lng", globalp.lng + count * 0.0005) \
                    .field("group", group) \
                    .tag("robot_name", robot_name) \
                    .time(get_nanosec(), WritePrecision.NS)  # need to put point in different time
                self.send_point(point)
            count+=1

    def image_callback(self, interval, direction):
        @throttle(interval)
        def inner_func(msg):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            retval, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            jpg_as_text = base64.b64encode(buffer).decode()
            for robot_name in self.robot_names:
                point = Point("image") \
                    .field("data", jpg_as_text) \
                    .tag("format", "jpeg") \
                    .tag("direction", direction) \
                    .tag("robot_name", robot_name) \
                    .time(get_nanosec(msg.header.stamp), WritePrecision.NS)
                self.send_point(point)
        return inner_func

    def activity_log_callback(self, msg):
        self.get_logger().info(f"{msg}")
        for robot_name in self.robot_names:
            if msg.category == "tour-text":
                point = Point("tour") \
                    .field("data", msg.text) \
                    .tag("robot_name", robot_name) \
                    .time(get_nanosec(msg.header.stamp), WritePrecision.NS)
                self.send_point(point)
            if msg.category == "destination-text":
                point = Point("destination") \
                    .field("data", msg.text) \
                    .tag("robot_name", robot_name) \
                    .time(get_nanosec(msg.header.stamp), WritePrecision.NS)
                self.send_point(point)

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
