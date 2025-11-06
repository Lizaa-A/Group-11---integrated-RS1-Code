# telemetry_dashboard_gui.py
import sys
import os
import json
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from nav2_msgs.msg import ParticleCloud
from ament_index_python.packages import get_package_share_directory

from PySide6.QtWidgets import (
    QApplication, QWidget, QStackedWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QGridLayout, QLineEdit, QSizePolicy,
    QProgressBar, QFrame, QFileDialog
)
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor
from nav_msgs.msg import OccupancyGrid
from .gui_format import Styles
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import subprocess

# -----------------------------
# Telemetry Listener Node
# -----------------------------
class TelemetryListener(Node):
    def __init__(self):
        super().__init__('telemetry_listener')
        self.telemetry_data = {}
        self.latest_image = None
        self.bridge = CvBridge()

        # Telemetry subscription (JSON string snapshots)
        self.create_subscription(String, '/telemetry/snapshot', self.callback, 10)

        # PGM image subscription (mono8)
        self.create_subscription(Image, '/pgm/image', self.image_callback, 10)

    def callback(self, msg):
        try:
            self.telemetry_data = json.loads(msg.data)
            print("Received telemetry:", self.telemetry_data)  
        except Exception:
            self.get_logger().warn('Invalid JSON received on /telemetry/snapshot')

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            # store numpy grayscale image
            self.latest_image = img
            self.get_logger().info('PGM image received on /pgm/image')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert /pgm/image: {e}')

    def update_data(self, data_dict):
        """Manually update telemetry data for testing purposes."""
        self.telemetry_data = data_dict

# -----------------------------
# Header Widget
# -----------------------------
class Header(QWidget):
    def __init__(self, stacked_widget, telemetry_node):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.telemetry_node = telemetry_node
        self.setStyleSheet(f"background-color: {Styles.BACKGROUND_COLOR};")
        layout = QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        self.setLayout(layout)
        self.setFixedHeight(60)

        self.home_btn = QPushButton("Home")
        self.home_btn.setStyleSheet(Styles.BUTTON)
        self.home_btn.clicked.connect(lambda: stacked_widget.setCurrentWidget(stacked_widget.dashboard_screen))

        self.logout_btn = QPushButton("Logout")
        self.logout_btn.setStyleSheet(Styles.BUTTON)
        self.logout_btn.clicked.connect(lambda: stacked_widget.setCurrentWidget(stacked_widget.login_screen))

        self.task_status_light = QLabel("●")
        self.task_status_light.setStyleSheet(f"color: {Styles.TASK_LIGHT_OFF}; font-size: 16px;")


        self.task_status_label = QLabel("Task: N/A")
        self.task_status_label.setStyleSheet(Styles.LABEL)

        layout.addWidget(self.home_btn)
        layout.addWidget(self.logout_btn)
        layout.addStretch()
        layout.addWidget(self.task_status_light)
        layout.addWidget(self.task_status_label)

        # update periodically from telemetry node
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(1000)

    def update_telemetry(self):
        data = getattr(self.telemetry_node, "telemetry_data", {})
        task = data.get("task_status", "Idle")

        # pick a colour depending on task
        color_map = {
            "FSM - SCAN FOR GOAL CLEARANCES": "#489239",
            "FSM - NAV TO GOAL": "#5AB647",
            "FSM - NO GOAL CLEARANCES - PLEASE LOAD NEW MAP": "#722929",
            "FSM - REQUESTING NEXT GOAL": "#5AB647",
            "FSM - BAD SUNLIGHT": "#4B2412",
            "FSM - BAD SOIL": "#4B2412",
            "FSM - RAKING": "#945C46",
            "FSM - TANK REFILLED": "#2196F3",
            "FSM - TANK EMPTY": "#FF0000",
            "FSM - LOW SEED LEVEL" 
            "FSM - SEEDS EMPTY — requesting plant stop"
            "FSM - CHECKING CONDITIONS": "#489239",
            "FSM - BAD CONDITIONS": "#ff0000",
            "FSM - SEEDING": "#945C46",
            "FSM - COVERING": "#945C46",
            "FSM - IRRIGATING": "#679CD8",
        }

        color = color_map.get(task.strip(), "#cccccc")
        self.task_status_light.setText("●")
        self.task_status_light.setStyleSheet(f"color: {color}; font-size: 24px;")
        self.task_status_light.repaint()


        self.task_status_label.setText(f"Task: {task}")


# -----------------------------
# Login Screen
# -----------------------------
class LoginScreen(QWidget):
    def __init__(self, switch_callback):
        super().__init__()
        self.switch_callback = switch_callback
        self.setStyleSheet(f"background-color: {Styles.BACKGROUND_COLOR}; color: {Styles.TEXT_COLOR};")

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        self.setLayout(layout)

        title = QLabel("Login to BloomBot")
        title.setStyleSheet(Styles.HEADER)
        title.setAlignment(Qt.AlignCenter)

        self.username = QLineEdit()
        self.username.setPlaceholderText("Username")
        self.username.setStyleSheet(Styles.LOGIN_INPUT)
        self.password = QLineEdit()
        self.password.setPlaceholderText("Password")
        self.password.setEchoMode(QLineEdit.Password)
        self.password.setStyleSheet(Styles.LOGIN_INPUT)

        self.login_btn = QPushButton("Login")
        self.login_btn.setStyleSheet(Styles.BUTTON)
        self.login_btn.clicked.connect(self.check_login)

        self.error_label = QLabel("")
        self.error_label.setStyleSheet("color: red;")
        self.error_label.setAlignment(Qt.AlignCenter)

        layout.addWidget(title)
        layout.addSpacing(8)
        layout.addWidget(self.username)
        layout.addWidget(self.password)
        layout.addWidget(self.login_btn)
        layout.addWidget(self.error_label)

    def check_login(self):
        if self.username.text() == "bloombot" and self.password.text() == "robotics1!":
            self.error_label.setText("")
            self.switch_callback()
        else:
            self.error_label.setText("Invalid username or password")

# -----------------------------
# Base Screen
# -----------------------------
class BaseScreen(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.setStyleSheet(f"background-color: {Styles.BACKGROUND_COLOR}; color: {Styles.TEXT_COLOR};")
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # header consumes telemetry_node via stacked_widget
        self.header = Header(stacked_widget, stacked_widget.telemetry_node)
        self.layout.addWidget(self.header)


# -----------------------------
# Live Monitoring Screen 
# -----------------------------
class LiveMonitoringScreen(BaseScreen):
    def __init__(self, telemetry_node, stacked_widget):
        super().__init__(stacked_widget)
        self.telemetry_node = telemetry_node

        # ---- Layout setup ----
        content_layout = QVBoxLayout()
        content_layout.setContentsMargins(20, 20, 20, 20)
        content_layout.setSpacing(20)
        self.layout.addLayout(content_layout)

        # -------------------------------
        # WATER TANK
        # -------------------------------
        water_row = QHBoxLayout()
        self.water_label = QLabel("Water Tank:")
        self.water_label.setStyleSheet(Styles.LABEL)
        self.water_dot = QLabel("●")
        self.water_dot.setStyleSheet("color: gray; font-size: 18px;")
        self.water_value = QLabel("0 L")
        self.water_value.setStyleSheet(Styles.SECONDARY_TEXT_COLOR)
        water_row.addWidget(self.water_label)
        water_row.addWidget(self.water_dot)
        water_row.addWidget(self.water_value)
        water_row.addStretch()
        content_layout.addLayout(water_row)

        self.water_bar = QProgressBar()
        self.water_bar.setRange(0, 100)
        self.water_bar.setTextVisible(True)
        self.water_bar.setFormat("0%")
        self.water_bar.setFixedHeight(18)
        content_layout.addWidget(self.water_bar)

        # -------------------------------
        # SOIL MOISTURE
        # -------------------------------
        soil_row = QHBoxLayout()
        self.soil_label = QLabel("Soil Moisture:")
        self.soil_label.setStyleSheet(Styles.LABEL)
        self.soil_dot = QLabel("●")
        self.soil_dot.setStyleSheet("color: gray; font-size: 18px;")
        self.soil_value = QLabel("0 (placeholder)")
        self.soil_value.setStyleSheet(Styles.SECONDARY_TEXT_COLOR)
        soil_row.addWidget(self.soil_label)
        soil_row.addWidget(self.soil_dot)
        soil_row.addWidget(self.soil_value)
        soil_row.addStretch()
        content_layout.addLayout(soil_row)

        self.soil_bar = QProgressBar()
        self.soil_bar.setRange(0, 100)
        self.soil_bar.setTextVisible(True)
        self.soil_bar.setFormat("0%")
        self.soil_bar.setFixedHeight(18)
        content_layout.addWidget(self.soil_bar)

        # -------------------------------
        # SEED WEIGHT
        # -------------------------------
        seed_row = QHBoxLayout()
        self.seed_label = QLabel("Seed Weight:")
        self.seed_label.setStyleSheet(Styles.LABEL)
        self.seed_dot = QLabel("●")
        self.seed_dot.setStyleSheet("color: gray; font-size: 18px;")
        self.seed_value = QLabel("0 g")
        self.seed_value.setStyleSheet(Styles.SECONDARY_TEXT_COLOR)
        seed_row.addWidget(self.seed_label)
        seed_row.addWidget(self.seed_dot)
        seed_row.addWidget(self.seed_value)
        seed_row.addStretch()
        content_layout.addLayout(seed_row)

        self.seed_bar = QProgressBar()
        self.seed_bar.setRange(0, 100)
        self.seed_bar.setTextVisible(True)
        self.seed_bar.setFormat("0%")
        self.seed_bar.setFixedHeight(18)
        content_layout.addWidget(self.seed_bar)

        # -------------------------------
        # SUNLIGHT
        # -------------------------------
        sun_row = QHBoxLayout()
        self.sun_label = QLabel("Sunlight:")
        self.sun_label.setStyleSheet(Styles.LABEL)
        self.sun_dot = QLabel("●")
        self.sun_dot.setStyleSheet("color: gray; font-size: 18px;")
        sun_row.addWidget(self.sun_label)
        sun_row.addWidget(self.sun_dot)
        sun_row.addStretch()
        content_layout.addLayout(sun_row)

        # -------------------------------
        # Timer to refresh telemetry
        # -------------------------------
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_telemetry)
        self.update_timer.start(1000)  # every second

    # ======================================================
    # TELEMETRY UPDATE
    # ======================================================
    def update_telemetry(self):
        data = getattr(self.telemetry_node, "telemetry_data", {})
# changed
        # ----------------- WATER -----------------
        water = data.get("water_tank", {})
        water_percent = float(water.get("level_percent", 100.0))
        water_liters = float(water.get("level_liters", 10.0))
        water_low = bool(water.get("low", False))

        self.water_bar.setValue(int(water_percent))
        self.water_bar.setFormat(f"{water_percent:.1f}%")
        self.water_value.setText(f"{water_liters:.1f} L")

        # water light color
        water_color = "#e74c3c" if water_low else "#2ecc71"
        self.water_dot.setStyleSheet(f"color: {water_color}; font-size: 18px;")

        # ----------------- SOIL -----------------
        soil = data.get("soil_moisture", {})
        soil_raw = float(soil.get("raw", 0.0))
        soil_percent = min(100, max(0, (soil_raw / 800) * 100))

        self.soil_bar.setValue(int(soil_percent))
        self.soil_bar.setFormat(f"{soil_percent:.1f}%")
        self.soil_value.setText(f"{soil_raw:.1f} (placeholder)")

        # soil light color (below 400 = red)
        soil_color = "#e74c3c" if soil_raw < 400 else "#2ecc71"
        self.soil_dot.setStyleSheet(f"color: {soil_color}; font-size: 18px;")

        # ----------------- SEED -----------------
        seed = data.get("seed_hopper", data.get("seed_weight", {}))

        # Some producers may publish "level_percent" instead of "percent_full" — support both
        seed_percent = float(seed.get("percent_full", seed.get("level_percent", 100.0)))
        seed_weight  = float(seed.get("weight_g", 100.0))
        seed_low     = bool(seed.get("low", False))
        seed_empty   = bool(seed.get("empty", False)) 

        # Progress bar + labels
        self.seed_bar.setValue(int(round(seed_percent)))
        self.seed_bar.setFormat(f"{seed_percent:.1f}%")
        self.seed_value.setText(f"{seed_weight:.1f} g")

        # Light colour: red if low OR empty, green otherwise
        seed_colour = "#e74c3c" if (seed_low or seed_empty) else "#2ecc71"
        self.seed_dot.setStyleSheet(f"color: {seed_colour}; font-size: 18px;")

        # ----------------- SUNLIGHT -----------------
        sun_ok = bool(data.get("sunlight_ok", False))
        sun_color = "#2ecc71" if sun_ok else "#e74c3c"
        self.sun_dot.setStyleSheet(f"color: {sun_color}; font-size: 18px;")


# -----------------------------
# Other Screens (placeholders)
# -----------------------------
class RobotsScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)

        layout = QVBoxLayout()
        self.layout.addLayout(layout)

        # Title
        title_label = QLabel("Robots")
        title_label.setStyleSheet(Styles.HEADER)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Husky image
        image_label = QLabel()
        image_label.setAlignment(Qt.AlignCenter)

        image_path = os.path.join(
            get_package_share_directory('ugv_dashboard_gui'),
            'test_images',
            'husky.png'
        )
        pixmap = QPixmap(image_path)
        if pixmap.isNull():
            image_label.setText("Husky image not found")
        else:
            image_label.setPixmap(pixmap.scaled(400, 250, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        layout.addWidget(image_label)

        # Husky info paragraph
        info_label = QLabel(
            "The Husky UGV is a versatile unmanned ground vehicle suitable for research, "
            "mapping, and field automation tasks. It can carry sensors, navigate autonomously, "
            "and integrate with ROS 2 for telemetry and control."
        )
        info_label.setWordWrap(True)
        info_label.setAlignment(Qt.AlignTop)
        info_label.setStyleSheet(f"{Styles.LABEL} padding: 16px; border-radius: 8px; background-color: #252834;")
        layout.addWidget(info_label)


class MappingScreen(BaseScreen):
    def __init__(self, stacked_widget: QStackedWidget, telemetry_node):
        super().__init__(stacked_widget)

        self.node = telemetry_node

        # ---------- Title ----------
        title_label = QLabel("Mapping Screen")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; color: white;")
        self.layout.addWidget(title_label)

        # ---------- Image Container ----------
        self.image_container = QWidget()
        self.image_container.setStyleSheet("background-color: #101010; border: 1px solid #444;")
        image_layout = QVBoxLayout(self.image_container)
        image_layout.setContentsMargins(10, 10, 10, 10)
        image_layout.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_container, stretch=1)

        # ---------- Image Display Label ----------
        self.image_label = QLabel("Waiting for map data...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setStyleSheet("background-color: #202020; color: white; border: 2px dashed #555;")
        image_layout.addWidget(self.image_label)

        # ---------- Status Label ----------
        self.status_label = QLabel("⏳ Connecting to /map ...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: gray; font-size: 12px;")
        self.layout.addWidget(self.status_label)

        # ---------- Storage ----------
        self.map_data = None
        self.map_info = None
        self.robot_pose = None
        self.particles = []  # list of (x, y) particle positions

        # ---------- ROS2 Subscriptions ----------

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            qos
        )

        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10
        )

        self.particle_sub = self.node.create_subscription(
            ParticleCloud,
            "/particle_cloud",
            self.particle_callback,
            10
        )

        # ---------- Update Timer ----------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map)
        self.timer.start(200)  # update every 0.2s

        print("MappingScreen initialized and waiting for map/particle data...")

    # ------------------- Map Callback -------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Remap occupancy to GUI-friendly PGM style
        # -1 unknown -> medium gray 128
        # 0 free -> light gray 200
        # >0 occupied -> dark gray 50
        display = np.zeros_like(data, dtype=np.uint8)
        display[data < 0] = 128
        display[data == 0] = 200
        display[data > 0] = 50

        # Flip Y-axis to match coordinate frames
        self.map_data = np.flipud(display)
        self.status_label.setText("Receiving /map data")

    # ------------------- Pose Callback -------------------
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg.pose.pose

    # ------------------- Particle Callback -------------------
    def particle_callback(self, msg: ParticleCloud):        
        self.particles = [(p.x, p.y) for p in msg.particles]

    # ------------------- GUI Update -------------------
    def update_map(self):
        if self.map_data is None:
            self.image_label.setText("Waiting for map data...")
            return

        height, width = self.map_data.shape

        # QImage creation
        qt_image = QImage(self.map_data.tobytes(), width, height, QImage.Format_Grayscale8)
        pixmap = QPixmap.fromImage(qt_image).scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )

        painter = QPainter(pixmap)
        x_scale = pixmap.width() / width
        y_scale = pixmap.height() / height

        # ---------- Draw Particles ----------
        if self.particles and self.map_info is not None:
            painter.setBrush(QColor(0, 255, 0, 180))
            painter.setPen(Qt.NoPen)
            for px, py in self.particles:
                mx = int((px - self.map_info.origin.position.x) / self.map_info.resolution)
                my = int((py - self.map_info.origin.position.y) / self.map_info.resolution)
                my = height - my
                painter.drawEllipse(int(mx * x_scale) - 2, int(my * y_scale) - 2, 4, 4)

        # ---------- Draw Robot ----------
        if self.robot_pose is not None and self.map_info is not None:
            painter.setBrush(QColor(255, 0, 0))
            painter.setPen(Qt.NoPen)
            mx = int((self.robot_pose.position.x - self.map_info.origin.position.x) / self.map_info.resolution)
            my = int((self.robot_pose.position.y - self.map_info.origin.position.y) / self.map_info.resolution)
            my = height - my
            painter.drawEllipse(int(mx * x_scale) - 3, int(my * y_scale) - 3, 6, 6)

        painter.end()
        self.image_label.setPixmap(pixmap)



class ManualScreen(BaseScreen):
    def __init__(self, stacked_widget: QStackedWidget, telemetry_node):
        super().__init__(stacked_widget)

        self.node = telemetry_node

        # ---------- Title ----------
        title_label = QLabel("Manual Piloting Screen")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; color: white;")
        self.layout.addWidget(title_label)

        # ---------- Image Container ----------
        self.image_container = QWidget()
        self.image_container.setStyleSheet("background-color: #101010; border: 1px solid #444;")
        image_layout = QVBoxLayout(self.image_container)
        image_layout.setContentsMargins(10, 10, 10, 10)
        image_layout.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_container, stretch=1)

        # ---------- Image Display Label ----------
        self.image_label = QLabel("Waiting for camera feed...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setStyleSheet(
            "background-color: #202020; color: white; border: 2px dashed #555;"
        )
        image_layout.addWidget(self.image_label)

        # ---------- Status Label ----------
        self.status_label = QLabel("Connecting to /camera/image ...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: gray; font-size: 12px;")
        self.layout.addWidget(self.status_label)

        # ---------- CV Bridge & Storage ----------
        self.bridge = CvBridge()
        self.latest_frame = None

        # ---------- ROS2 Subscription ----------
        self.subscription = self.node.create_subscription(
            Image,
            "/camera/image",
            self.image_callback,
            10
        )

        # ---------- ROS Publisher for Manual Piloting ----------
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # ---------- Update Timer ----------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(100)  # refresh every 0.1s

        # ---------- Enable Keyboard Focus ----------
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

    # ------------------- ROS Callback -------------------
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = cv_image
            self.status_label.setText("Receiving /camera/image frames")
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {e}")
            self.status_label.setText(f" Image conversion error: {e}")

    # ------------------- GUI Update -------------------
    def update_image(self):
        if self.latest_frame is not None:
            rgb_image = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(
                pixmap.scaled(
                    self.image_label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
            )
        else:
            self.image_label.setText("Waiting for camera feed...")

    # ------------------- Keyboard Control -------------------
    def keyPressEvent(self, event):
        twist = Twist()
        if event.key() == Qt.Key_Up:
            twist.linear.x = 0.2
        elif event.key() == Qt.Key_Down:
            twist.linear.x = -0.2
        elif event.key() == Qt.Key_Left:
            twist.angular.z = 0.5
        elif event.key() == Qt.Key_Right:
            twist.angular.z = -0.5
        self.cmd_pub.publish(twist)

    def keyReleaseEvent(self, event):
        twist = Twist()  # Stop the robot
        self.cmd_pub.publish(twist)


# -----------------------------
# Dashboard Screen (buttons -> screens)
# -----------------------------
class DashboardScreen(BaseScreen):
    def __init__(self, telemetry_node, stacked_widget):
        super().__init__(stacked_widget)
        self.telemetry_node = telemetry_node

        grid = QGridLayout()
        grid.setSpacing(20)
        self.layout.addLayout(grid)

        # --- Top row ---
        welcome_label = QLabel("Welcome to BloomBot!")
        welcome_label.setStyleSheet(Styles.HEADER)
        welcome_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        grid.addWidget(welcome_label, 0, 0)

        description_label = QLabel("   AUTONOMOUS PLANTING SYSTEM")
        description_label.setWordWrap(True)
        description_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        description_label.setStyleSheet(Styles.LABEL)
        grid.addWidget(description_label, 0, 1)

        # --- Middle row ---
        # Left: logo image
        image_label = QLabel()
        image_label.setAlignment(Qt.AlignCenter)

        try:
            pkg_path = get_package_share_directory('ugv_dashboard_gui')
            image_path = os.path.join(pkg_path, 'test_images', 'BloomBot_logo.png')
            if os.path.exists(image_path):
                pixmap = QPixmap(image_path)
                image_label.setPixmap(
                    pixmap.scaled(300, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
            else:
                image_label.setText("Logo not found ")
                image_label.setStyleSheet("color: gray; font-size: 14px;")
        except Exception as e:
            image_label.setText(f"Error loading logo: {e}")
            image_label.setStyleSheet("color: red; font-size: 12px;")

        grid.addWidget(image_label, 1, 0)

        # Right: info box
        info_box = QLabel(
            "BloomBot integrates robotics, mapping, and real-time monitoring to automate field maintenance tasks such as seeding, watering, and soil analysis. "
            "This dashboard allows you to monitor live data, plan missions, and control robots in the field environment."
        )
        info_box.setWordWrap(True)
        info_box.setAlignment(Qt.AlignTop)
        info_box.setStyleSheet(f"{Styles.LABEL} padding: 16px; border-radius: 8px; background-color: #252834;")
        grid.addWidget(info_box, 1, 1)

        # --- Bottom row ---
        # Left: navigation buttons
        buttons_layout = QVBoxLayout()
        self.robot_btn = QPushButton("Robots")
        self.live_btn = QPushButton("Live Monitoring")
        self.mapping_btn = QPushButton("Mapping")
        self.manual_btn = QPushButton("Manual Piloting")

        for btn in [self.robot_btn, self.live_btn, self.mapping_btn, self.manual_btn]:
            btn.setStyleSheet(Styles.BUTTON)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_layout.addWidget(btn)
        grid.addLayout(buttons_layout, 2, 0)

        # Right: short descriptions corresponding to buttons
        desc_layout = QVBoxLayout()
        desc_texts = [
            "• View robot information",
            "• Monitor live sensor data",
            "• View lidar mapping and active robot position",
            "• Manually pilot robots"
        ]
        for text in desc_texts:
            lbl = QLabel(text)
            lbl.setStyleSheet(f"{Styles.LABEL} padding-left: 8px;")
            desc_layout.addWidget(lbl)
        grid.addLayout(desc_layout, 2, 1)

        # --- Stretching ---
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 2)
        grid.setRowStretch(2, 2)

        # --- Screens ---
        self.live_monitoring_screen = LiveMonitoringScreen(telemetry_node, stacked_widget)
        self.robots_screen = RobotsScreen(stacked_widget)
        self.mapping_screen = MappingScreen(stacked_widget, telemetry_node)
        self.manual_screen = ManualScreen(stacked_widget, telemetry_node)

        for screen in [
            self.live_monitoring_screen,
            self.robots_screen,
            self.mapping_screen,
            self.manual_screen
        ]:
            self.stacked_widget.addWidget(screen)

        # --- Button connections ---
        self.live_btn.clicked.connect(lambda: self.stacked_widget.setCurrentWidget(self.live_monitoring_screen))
        self.robot_btn.clicked.connect(lambda: self.stacked_widget.setCurrentWidget(self.robots_screen))
        self.mapping_btn.clicked.connect(lambda: self.stacked_widget.setCurrentWidget(self.mapping_screen))
        self.manual_btn.clicked.connect(lambda: self.stacked_widget.setCurrentWidget(self.manual_screen))


# -----------------------------
# Main App
# -----------------------------
class BloomBotApp(QStackedWidget):
    def __init__(self, telemetry_node):
        super().__init__()
        self.telemetry_node = telemetry_node
        self.setStyleSheet(f"background-color: {Styles.BACKGROUND_COLOR}; color: {Styles.TEXT_COLOR};")
        self.login_screen = LoginScreen(self.show_dashboard)
        self.dashboard_screen = DashboardScreen(telemetry_node, self)

        self.addWidget(self.login_screen)
        self.addWidget(self.dashboard_screen)
        self.setCurrentWidget(self.login_screen)

    def show_dashboard(self):
        self.setCurrentWidget(self.dashboard_screen)

# -----------------------------
# Main entry point
# -----------------------------
def main():
    rclpy.init(args=None)
    telemetry_node = TelemetryListener()
    # Start rclpy spinning in a background thread (so Qt mainloop runs on main thread)
    thread = Thread(target=lambda: rclpy.spin(telemetry_node), daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    app.setStyleSheet(f"QWidget {{ background-color: {Styles.BACKGROUND_COLOR}; color: {Styles.TEXT_COLOR}; }}")
    main_window = BloomBotApp(telemetry_node)
    main_window.resize(1100, 720)
    main_window.show()

    # Optional test data so UI shows something before real telemetry arrives
    telemetry_node.update_data({
        "task_status": "Idle",
        "water_tank": {"level_liters": 5.0, "level_percent": 50.0, "low": False},
        "soil_moisture": {"raw": 00.0, "percent": 0.0},
        "sunlight_ok": True
    })

    # run the Qt app (blocks until closed)
    exit_code = app.exec()

    # cleanup
    try:
        telemetry_node.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
