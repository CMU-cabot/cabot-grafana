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

import rclpy
from rclpy.node import Node
from cabot_msgs.msg import PoseLog
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime


class ClientNode(Node):
    def __init__(self):
        super().__init__("client_node")

        self.host = self.declare_parameter("host", "").value
        self.token = self.declare_parameter("token", "").value
        self.org = self.declare_parameter("org", "").value
        self.bucket = self.declare_parameter("bucket", "").value

        self.get_logger().info(f"{[self.host, self.token, self.org, self.bucket]}")

        self.client = InfluxDBClient(url=self.host, token=self.token, org=self.org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

        self.pose_log_sub = self.create_subscription(PoseLog, '/cabot/pose_log', self.pose_log_callback, 10)
        self.diag_agg_sub = self.create_subscription(DiagnosticArray, '/diagnostics_agg', self.diagnostics_callback, 10)

        
    def pose_log_callback(self, msg):
        point = Point("pose_data").field("lat", msg.lat).field("lng", msg.lng).field("floor", msg.floor).time(datetime.utcnow())
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


def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
