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
from cabot_msgs.msg import PoseLog
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64

from cabot_ui import geoutil, cabot_rclpy_util

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime
import time

class Throttle():
    def __init__(self, interval):
        self.interval = interval
        self.last = time.time()

    def check(self):
        return time.time() > self.last + self.interval

    def reset(self):
        self.last = time.time()


class ClientNode(Node):
    def __init__(self):
        super().__init__("client_node")
        cabot_rclpy_util.CaBotRclpyUtil.initialize(self)

        self.host = self.declare_parameter("host", "").value
        self.token = self.declare_parameter("token", "").value
        self.org = self.declare_parameter("org", "").value
        self.bucket = self.declare_parameter("bucket", "").value
        anchor_file = self.declare_parameter("anchor_file", "").value
        self.anchor = None
        self.get_logger().info(F"Anchor file is {anchor_file}")
        if anchor_file is not None:
            temp = geoutil.get_anchor(anchor_file)
            if temp is not None:
                self.anchor = temp
            else:
                self.get_logger.warn(F"could not load anchor_file \"{anchor_file}\"")

        self.client = InfluxDBClient(url=self.host, token=self.token, org=self.org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

        self.pose_log_sub = self.create_subscription(PoseLog, '/cabot/pose_log', self.pose_log_callback, 10)
        self.pose_log_throttle = Throttle(1.0)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cabot/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_throttle = Throttle(0.2)
        
        self.diag_agg_sub = self.create_subscription(DiagnosticArray, '/diagnostics_agg', self.diagnostics_callback, 10)

        self.plan_sub = self.create_subscription(Path, '/path', self.path_callback, 10)

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)        
        self.image_throttle = Throttle(10)
        self.bridge = CvBridge()

        self.current_floor = 0


    def pose_log_callback(self, msg):
        if not self.pose_log_throttle.check():
            return
        self.pose_log_throttle.reset()
        orientation = msg.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        point = Point("pose_data") \
            .field("lat", msg.lat) \
            .field("lng", msg.lng) \
            .field("floor", msg.floor) \
            .field("yaw", - self.anchor.rotate - yaw/math.pi*180) \
            .time(datetime.utcnow())
        self.current_floor = msg.floor
        self.write_api.write(bucket=self.bucket, org=self.org, record=point)

    def cmd_vel_callback(self, msg):
        if not self.cmd_vel_throttle.check():
            return
        self.cmd_vel_throttle.reset()
        v = math.sqrt(msg.linear.x * msg.linear.x + msg.linear.y * msg.linear.y + msg.linear.z * msg.linear.z)
        point = Point("velocity") \
            .field("value", v) \
            .time(datetime.utcnow())
        self.write_api.write(bucket=self.bucket, org=self.org, record=point)

    def diagnostics_callback(self, msg):
        target_name = "Soft: Navigation2"
        max_level = 0
        for state in msg.status:
            items = state.name.split("/")
            if len(items) < 3:
                continue
            if items[2] == target_name:
                level = int.from_bytes(state.level, byteorder='big')
                if max_level < level:
                    max_level = level
        point = Point("diagnostic")\
            .tag("name", target_name)\
            .field("level", max_level)\
            .time(datetime.utcnow())
        self.write_api.write(bucket=self.bucket, org=self.org, record=point)

    def path_callback(self, msg):
        for pose in msg.poses:
            position = pose.pose.position
            localp = geoutil.Point(x=position.x, y=position.y)
            globalp = geoutil.local2global(localp, self.anchor)
            self.get_logger().info(f"{position}")
            self.get_logger().info(f"{localp} {self.anchor}")
            self.get_logger().info(f"{globalp}")
            
            point = Point("plan") \
                .field("lat", globalp.lat) \
                .field("lng", globalp.lng) \
                .time(datetime.utcnow())
            self.write_api.write(bucket=self.bucket, org=self.org, record=point)

    def image_callback(self, msg):
        if not self.image_throttle.check():
            return
        self.image_throttle.reset()

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        retval, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        jpg_as_text = base64.b64encode(buffer).decode()
        point = Point("image").tag("format", "jpeg").field("data", jpg_as_text)
        self.write_api.write(bucket=self.bucket, org=self.org, record=point)


def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
