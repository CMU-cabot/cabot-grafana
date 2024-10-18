#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Image, BatteryState
import logging
from influxdb_client import InfluxDBClient, Point, WriteOptions
from influxdb_client.client.write_api import SYNCHRONOUS


logging.basicConfig(level=logging.DEBUG)

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

class BattryListener(Node):
    def __init__(self):
        super()._init_('battey_log_listener')
        self.subscription = self.create_subscription(BatteryState, '/battery_state', self.battery_log_callback, 10)
        self.client = InfluxDBClient(url='http://localhost:8086', token=token, org=org)
        self.write_api = self.client.write_api(write_option=SYNCHRONOUS)
     
    def battery_log_callback(self, msg):
        point = Point("battry").tag("robot_name", robot_name).field("percentage", msg.percentag).time(datetime.utcnow())
        write_api.write(bucket=bucket, org=org, record=point)

    def __del__(self):        
        self.client.close() 


def main(arg=None):
    rclpy.init(args=args)
    battry_log_listener = BattryListener()
    rclpy.spin(battry_log_listener)
    battry_log_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
