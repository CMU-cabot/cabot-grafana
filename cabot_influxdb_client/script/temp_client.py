#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Temperature
import logging
from influxdb_client import InfluxDBClient, Point
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

class TemperatureListener(Node):

    def __init__(self):
        super().__init__('temperature_log_listener')
        self.subscription = self.create_subscription(
            Temperature,
            'cabot/temperature1',
            self.temp_log_callback1,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("node ready")

    def temp_log_callback1(self, msg):
        self.get_logger().info(f"Received temperature: {msg.temperature}")
        point = Point("temperature").tag("unit", "celsius").field("value", msg.temperature).time(datetime.utcnow())
        try:
            write_api.write(bucket=bucket, org=org, record=point)
        except:
            pass

    def __init__(self):
        super().__init__('temperature_log_listener')
        self.subscription = self.create_subscription(
            Temperature,
            'cabot/temperature2',
            self.temp_log_callback2,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("node ready")

    def temp_log_callback2(self, msg):
        self.get_logger().info(f"Received temperature: {msg.temperature}")
        point = Point("temperature").tag("unit", "celsius").field("value", msg.temperature).time(datetime.utcnow())
        try:
            write_api.write(bucket=bucket, org=org, record=point)
        except:
            pass

    def __init__(self):
        super().__init__('temperature_log_listener')
        self.subscription = self.create_subscription(
            Temperature,
            'cabot/temperature3',
            self.temp_log_callback3,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("node ready")

    def temp_log_callback3(self, msg):
        self.get_logger().info(f"Received temperature: {msg.temperature}")
        point = Point("temperature").tag("unit", "celsius").field("value", msg.temperature).time(datetime.utcnow())
        try:
            write_api.write(bucket=bucket, org=org, record=point)
        except:
            pass

    def __init__(self):
        super().__init__('temperature_log_listener')
        self.subscription = self.create_subscription(
            Temperature,
            'cabot/temperature4',
            self.temp_log_callback4,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("node ready")

    def temp_log_callback4(self, msg):
        self.get_logger().info(f"Received temperature: {msg.temperature}")
        point = Point("temperature").tag("unit", "celsius").field("value", msg.temperature).time(datetime.utcnow())
        try:
            write_api.write(bucket=bucket, org=org, record=point)
        except:
            pass

    def __init__(self):
        super().__init__('temperature_log_listener')
        self.subscription = self.create_subscription(
            Temperature,
            'cabot/temperature5',
            self.temp_log_callback5,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("node ready")

    def temp_log_callback5(self, msg):
        self.get_logger().info(f"Received temperature: {msg.temperature}")
        point = Point("temperature").tag("unit", "celsius").field("value", msg.temperature).time(datetime.utcnow())
        try:
            write_api.write(bucket=bucket, org=org, record=point)
        except:
            pass



def main(args=None):
    rclpy.init(args=args)
    temperature_subscriber = TemperatureListener()
    rclpy.spin(temperature_log_listener)
    temperature_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
