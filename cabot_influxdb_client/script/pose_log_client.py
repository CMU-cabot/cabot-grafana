#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime
from cabot_msgs.msg import PoseLog
from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import SYNCHRONOUS


USERNAME = "cabot"
PASSWORD = "cabot-influxdb"
TOKEN = "a54a87f7-73a0-4534-9741-ad7ff4e7d111"
ORGANIZATION_NAME = "cabot"
BUCKET_NAME = "grafana-test"
host = "localhost"
port = 8086

influxdb_url = f"http://{host}:{port}"
token = TOKEN
org = ORGANIZATION_NAME
bucket = BUCKET_NAME

client = InfluxDBClient(url=influxdb_url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)


class PoseLogListener(Node):
    def __init__(self):
        super().__init__('pose_log_listener')
        self.subscription = self.create_subscription(PoseLog, '/cabot/pose_log', self.pose_log_callback, 10)
        self.client = InfluxDBClient(url='http://localhost:8086', token=token, org=org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

    def pose_log_callback(self, msg):
        point = Point("pose_data").field("lat", msg.lat).field("lng", msg.lng).field("floor", msg.floor).time(datetime.utcnow())
        self.write_api.write(bucket=bucket, org=org, record=point)

    def __del__(self):
        self.client.close()

def main(args=None):
    rclpy.init(args=args)
    pose_log_listener = PoseLogListener()
    rclpy.spin(pose_log_listener)
    pose_log_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
