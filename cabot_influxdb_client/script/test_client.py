#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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

class TemperatureSubscriber(Node):

    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Temperature,
            'temparature',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("node ready")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received temperature: {msg.temperature}")
        point = Point("temperature").tag("unit", "celsius").field("value", msg.temperature)
        try:
            write_api.write(bucket=bucket, org=org, record=point)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    temperature_subscriber = TemperatureSubscriber()
    rclpy.spin(temperature_subscriber)
    temperature_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
