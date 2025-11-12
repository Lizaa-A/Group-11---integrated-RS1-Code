#!/usr/bin/env python3
import socket, threading, re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool

def adc_to_percent(raw: int, dry: int, wet: int) -> float:
    if dry == wet: return 0.0
    pct = (raw - dry) * 100.0 / (wet - dry)
    return 0.0 if pct < 0 else 100.0 if pct > 100 else pct

class SinglePortTcpToRos(Node):
    def __init__(self):
        super().__init__('tcp_server_to_ros2_sensors')

        # Params
        self.listen_host = self.declare_parameter('listen_host', '0.0.0.0').value
        self.port        = int(self.declare_parameter('port', 5005).value)
        self.sun_thresh  = int(self.declare_parameter('sun_raw_threshold', 15).value)
        self.dry_adc     = int(self.declare_parameter('dry_adc', 150).value)
        self.wet_adc     = int(self.declare_parameter('wet_adc', 400).value)

        # Publishers
        self.pub_sun_raw  = self.create_publisher(Int32,  '/sunlight/raw', 10)
        self.pub_sun_ok   = self.create_publisher(Bool,   '/sunlight/ok',  10)
        self.pub_soil_raw = self.create_publisher(Int32,  '/soil/moisture_raw', 10)
        self.pub_soil_pct = self.create_publisher(Float32,'/soil/moisture_1',   10)

        # One TCP server
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((self.listen_host, self.port))
        self.srv.listen(5)
        self.get_logger().info(f"[SENSORS] Listening {self.listen_host}:{self.port}")

        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self.srv.accept()
                self.get_logger().info(f"Client connected {addr}")
                threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()
            except Exception as e:
                self.get_logger().warn(f"Accept error: {e}")

    def _client_loop(self, conn):
        buf = b""
        try:
            while rclpy.ok():
                chunk = conn.recv(4096)
                if not chunk:
                    self.get_logger().warn("Client disconnected (EOF)")
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    s = line.decode('utf-8', errors='ignore').strip()
                    if not s: 
                        continue
                    m = re.match(r'^\s*(SUN|SOIL)\s*:\s*(-?\d+)\s*$', s, re.IGNORECASE)
                    if not m:
                        n = self._parse_any_int(s)
                        if n is None:
                            self.get_logger().warn(f"Ignoring non-int line: {s!r}")
                            continue
                        self._handle_soil_value(n)
                        continue

                    tag = m.group(1).upper()
                    val = int(m.group(2))
                    if tag == 'SUN':
                        self._handle_sun_value(val)
                    else:
                        self._handle_soil_value(val)
        except Exception as e:
            self.get_logger().warn(f"Client loop error: {e}")
        finally:
            try: conn.close()
            except: pass

    def _parse_any_int(self, s: str):
        try:
            return int(s.strip())
        except ValueError:
            m = re.search(r'(-?\d+)', s)
            return int(m.group(1)) if m else None

    def _handle_sun_value(self, val: int):
        self.pub_sun_raw.publish(Int32(data=val))
        self.pub_sun_ok .publish(Bool (data=(val >= self.sun_thresh)))

    def _handle_soil_value(self, val: int):
        self.pub_soil_raw.publish(Int32(data=val))
        pct = float(adc_to_percent(val, self.dry_adc, self.wet_adc))
        self.pub_soil_pct.publish(Float32(data=pct))

def main():
    rclpy.init()
    node = SinglePortTcpToRos()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()