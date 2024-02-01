#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import SYNCHRONOUS

from rosidl_runtime_py import message_to_yaml

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


def getLevelText(level):
    if level == DiagnosticStatus.OK:
        return "OK"
    elif level == DiagnosticStatus.WARN:
        return "WARN"
    elif level == DiagnosticStatus.ERROR:
        return "ERROR"
    elif level == DiagnosticStatus.STALE:
        return "STALE"
    else:
        return "UNKNOWN"


class DiagnosticsListener(Node):
    def __init__(self):
        super().__init__('pose_log_listener')
        self.subscription = self.create_subscription(DiagnosticArray, '/diagnostics_agg', self.diagnostics_callback, 10)
        self.subscription2 = self.create_subscription(DiagnosticStatus, '/diagnostics_toplevel_state', self.diagnostics2_callback, 10)
        self.client = InfluxDBClient(url='http://localhost:8086', token=token, org=org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

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
        level_text = getLevelText(max_level)
        point = Point("diagnostic")\
            .tag("name", target_name)\
            .field("level", max_level)\
            .field("level_text", level_text)\
            .time(datetime.utcnow())
        self.write_api.write(bucket=bucket, org=org, record=point)

    def diagnostics2_callback(self, msg):
        pass
        #print(message_to_yaml(msg))
        #point = Point("pose_data").field("lat", msg.lat).field("lng", msg.lng).field("floor", msg.floor).time(datetime.utcnow())
        #self.write_api.write(bucket=bucket, org=org, record=point)

    def __del__(self):
        self.client.close()

def main(args=None):
    rclpy.init(args=args)
    pose_log_listener = DiagnosticsListener()
    rclpy.spin(pose_log_listener)
    pose_log_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
