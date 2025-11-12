#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
import json

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')


        self.telemetry_data = {
            "task_status": "Idle",
            "soil_moisture": {"raw": 0},
            "sunlight_ok": False,
            "water_tank": {"level_liters": 0, "level_percent": 0, "low": False},
            "seed_hopper": {
                "weight_g": 0,          # Int grams
                "percent_full": 0,      # Int percent 0..100
                "low": False,           # Bool - low threshold crossed
                "empty": False          # Bool - derived: percent_full <= 0
            }
        }

        self.pub = self.create_publisher(String, '/telemetry/snapshot', 10)

        self.create_subscription(Int32, '/soil/moisture_raw', self.soil_callback, 10)
        self.create_subscription(Bool,  '/sunlight/ok',       self.sunlight_callback, 10)

        self.create_subscription(Int32, '/water_tank/level_percent', self.water_level_callback, 10)
        self.create_subscription(Int32, '/water_tank/volume_l',      self.water_volume_callback, 10)
        self.create_subscription(Bool,  '/water_tank/low',           self.water_low_callback, 10)

        self.create_subscription(String, '/mission/fsm_status', self.status_callback, 10)

        self.create_subscription(Int32, '/seed_hopper/weight_g',      self.seed_weight_callback, 10)
        self.create_subscription(Int32, '/seed_hopper/level_percent', self.seed_percent_callback, 10)
        self.create_subscription(Bool,  '/seed_hopper/low',           self.seed_low_callback, 10)

        self.get_logger().info("Telemetry node initialized and waiting for sensor data...")

    def soil_callback(self, msg: Int32):
        self.telemetry_data["soil_moisture"]["raw"] = int(msg.data)
        self.publish_telemetry()

    def sunlight_callback(self, msg: Bool):
        self.telemetry_data["sunlight_ok"] = bool(msg.data)
        self.publish_telemetry()

    def water_level_callback(self, msg: Int32):
        self.telemetry_data["water_tank"]["level_percent"] = int(msg.data)
        self.publish_telemetry()

    def water_volume_callback(self, msg: Int32):
        self.telemetry_data["water_tank"]["level_liters"] = int(msg.data)
        self.publish_telemetry()

    def water_low_callback(self, msg: Bool):
        self.telemetry_data["water_tank"]["low"] = bool(msg.data)
        self.publish_telemetry()

    def status_callback(self, msg: String):
        self.telemetry_data["task_status"] = msg.data
        self.publish_telemetry()

    def seed_weight_callback(self, msg: Int32):
        self.telemetry_data["seed_hopper"]["weight_g"] = int(msg.data)
        self.publish_telemetry()

    def seed_percent_callback(self, msg: Int32):
        pct = int(msg.data)
        self.telemetry_data["seed_hopper"]["percent_full"] = pct
        self.telemetry_data["seed_hopper"]["empty"] = (pct <= 0)
        self.publish_telemetry()

    def seed_low_callback(self, msg: Bool):
        self.telemetry_data["seed_hopper"]["low"] = bool(msg.data)
        self.publish_telemetry()

    def publish_telemetry(self):
        msg = String()
        msg.data = json.dumps(self.telemetry_data)
        self.pub.publish(msg)
        self.get_logger().info(f"Published telemetry: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Telemetry node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()