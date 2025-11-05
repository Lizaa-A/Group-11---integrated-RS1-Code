# telemetry_dashboard_gui.py
import sys
import os
import json
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from PySide6.QtWidgets import (
    QApplication, QWidget, QStackedWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QGridLayout, QLineEdit, QSizePolicy,
    QProgressBar, QFrame, QFileDialog
)
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QImage, QPixmap

from .gui_format import Styles

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
            "Idle": "#cccccc",
            "Planting": "#4CAF50",
            "Watering": "#2196F3",
            "Harvesting": "#FF9800",
            "Error": "#FF4D4D"
        }

        color = color_map.get(task, "#cccccc")

        # ✅ update the small coloured light
        self.task_status_light.setStyleSheet(f"background-color: {color}; border-radius: 6px;")

        # ✅ update the label beside it
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
# Live Monitoring Screen (left sensors, right PGM)
# -----------------------------
# -----------------------------
# Live Monitoring Screen (with water bar)
# -----------------------------
class LiveMonitoringScreen(BaseScreen):
    def __init__(self, telemetry_node, stacked_widget):
        super().__init__(stacked_widget)
        self.telemetry_node = telemetry_node

        content_layout = QHBoxLayout()
        content_layout.setContentsMargins(12, 12, 12, 12)
        self.layout.addLayout(content_layout)

        # Left: sensors stack
        left = QVBoxLayout()
        left.setSpacing(12)

        # Water row: label + status dot + progress bar + percent
        water_label_row = QHBoxLayout()
        self.water_label = QLabel("Water Tank:")
        self.water_label.setStyleSheet(Styles.LABEL)
        self.water_dot = QLabel("●")
        self.water_dot.setStyleSheet("color: gray; font-size: 18px;")
        water_label_row.addWidget(self.water_label)
        water_label_row.addWidget(self.water_dot)
        water_label_row.addStretch()
        left.addLayout(water_label_row)

        self.water_bar = QProgressBar()
        self.water_bar.setRange(0, 100)
        self.water_bar.setTextVisible(False)
        self.water_bar.setFixedHeight(16)
        self.water_percent = QLabel("0%")
        self.water_percent.setStyleSheet(Styles.SECONDARY_TEXT_COLOR)
        left.addWidget(self.water_bar)

        # Sunlight row: label + status dot
        sun_row = QHBoxLayout()
        self.sun_label = QLabel("Sunlight:")
        self.sun_label.setStyleSheet(Styles.LABEL)
        self.sun_dot = QLabel("●")
        self.sun_dot.setStyleSheet("color: gray; font-size: 18px;")
        sun_row.addWidget(self.sun_label)
        sun_row.addWidget(self.sun_dot)
        sun_row.addStretch()
        left.addLayout(sun_row)

        # Soil moisture: label + progress bar + percent
        soil_label = QLabel("Soil Moisture:")
        soil_label.setStyleSheet(Styles.LABEL)
        self.soil_bar = QProgressBar()
        self.soil_bar.setRange(0, 100)
        self.soil_bar.setTextVisible(False)
        self.soil_bar.setFixedHeight(16)
        self.soil_percent = QLabel("0%")
        self.soil_percent.setStyleSheet(Styles.SECONDARY_TEXT_COLOR)
        left.addWidget(soil_label)
        left.addWidget(self.soil_bar)

        # Add stretch
        left.addStretch()

        # Right: PGM image display + controls
        right = QVBoxLayout()
        right.setSpacing(8)

        # small controls: load default, optionally pick file
        controls_row = QHBoxLayout()
        self.load_default_btn = QPushButton("Load default map")
        self.load_default_btn.setStyleSheet(Styles.BUTTON)
        self.load_default_btn.clicked.connect(self.load_default_pgm)
        self.pick_btn = QPushButton("Pick map...")
        self.pick_btn.setStyleSheet(Styles.BUTTON)
        self.pick_btn.clicked.connect(self.pick_pgm_file)
        controls_row.addWidget(self.load_default_btn)
        controls_row.addWidget(self.pick_btn)
        controls_row.addStretch()
        right.addLayout(controls_row)

        # PGM display label
        self.pgm_label = QLabel()
        self.pgm_label.setFixedSize(420, 420)
        self.pgm_label.setStyleSheet("background-color: #000; border: 1px solid #555;")
        self.pgm_label.setAlignment(Qt.AlignCenter)
        right.addWidget(self.pgm_label, alignment=Qt.AlignTop)

        content_layout.addLayout(left, stretch=1)
        content_layout.addLayout(right, stretch=1)

        # timers
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_telemetry)
        self.update_timer.start(1000)

        self.pgm_timer = QTimer()
        self.pgm_timer.timeout.connect(self.update_pgm_display)
        self.pgm_timer.start(400)

        # load default on init
        self.load_default_pgm()

    def update_telemetry(self):
        data = getattr(self.telemetry_node, "telemetry_data", {})

        # ---- soil ----
        soil = data.get("soil_moisture", {})
        if isinstance(soil, dict):
            soil_percent = soil.get("percent", 0.0)
        else:
            soil_percent = float(soil) if soil is not None else 0.0
        soil_percent = max(0.0, min(100.0, float(soil_percent)))
        self.soil_bar.setValue(int(soil_percent))
        self.soil_percent.setText(f"{soil_percent:.1f}%")

        # ---- water ----
        water = data.get("water_tank", {})
        if isinstance(water, dict):
            water_percent = water.get("level_percent", 0.0)
            water_low = bool(water.get("low", False))
        else:
            water_percent = 0.0
            water_low = False

        # water progress bar + percent
        self.water_bar.setValue(int(max(0, min(100, water_percent))))
        self.water_percent.setText(f"{water_percent:.1f}%")

        # water dot color
        water_color = "#e74c3c" if water_low else "#2ecc71"
        self.water_dot.setStyleSheet(f"color: {water_color}; font-size: 18px;")

        # ---- sunlight ----
        sun_ok = bool(data.get("sunlight_ok", False))
        sun_color = "#2ecc71" if sun_ok else "#e74c3c"
        self.sun_dot.setStyleSheet(f"color: {sun_color}; font-size: 18px;")


    # -----------------------------
    # PGM Loading / Display helpers
    # -----------------------------
    def default_pgm_path(self):
        # base dir is package/ugv_dashboard_gui
        base_dir = os.path.dirname(os.path.dirname(__file__))
        return os.path.join(base_dir, "test_images", "siteA.pgm")

    def load_default_pgm(self):
        path = self.default_pgm_path()
        if os.path.exists(path):
            pixmap = QPixmap(path)
            self.set_pixmap_to_label(pixmap)
        else:
            self.pgm_label.setText("Default image not found")

    def pick_pgm_file(self):
        # file dialog relative to package folder
        start_dir = os.path.dirname(os.path.dirname(__file__))
        fname, _ = QFileDialog.getOpenFileName(self, "Select PGM image", start_dir, "PGM Files (*.pgm);;All Files (*)")
        if fname:
            pixmap = QPixmap(fname)
            self.set_pixmap_to_label(pixmap)

    def set_pixmap_to_label(self, pixmap: QPixmap):
        if pixmap.isNull():
            self.pgm_label.setText("Invalid image")
            return
        scaled = pixmap.scaled(self.pgm_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.pgm_label.setPixmap(scaled)

    def update_pgm_display(self):
        """If an image arrives on /pgm/image (as numpy mono8), display it."""
        img = getattr(self.telemetry_node, "latest_image", None)
        if img is None:
            return
        try:
            h, w = img.shape
            q_img = QImage(img.data, w, h, w, QImage.Format_Grayscale8)
            pix = QPixmap.fromImage(q_img)
            self.set_pixmap_to_label(pix)
        except Exception as e:
            print(f"[GUI] update_pgm_display error: {e}")

    # -----------------------------
    # Telemetry update
    # -----------------------------
    def update_telemetry(self):
        data = getattr(self.telemetry_node, "telemetry_data", {})

        # ----------------- Soil -----------------
        soil = data.get("soil_moisture", {})
        if isinstance(soil, dict):
            soil_percent = soil.get("percent", 0.0)
            soil_raw = soil.get("raw", 0.0)
        else:
            soil_percent = float(soil) if soil is not None else 0.0
            soil_raw = soil_percent

        soil_percent = max(0.0, min(100.0, soil_percent))
        self.soil_bar.setValue(int(soil_percent))
        self.soil_bar.setFormat(f"{soil_percent:.1f}%")  # Show % inside the bar
        self.soil_bar.setTextVisible(True)

        # Optional: update label above bar
        self.soil_label.setText(f"Soil: {soil_raw:.1f} (Raw)")

        # ----------------- Water -----------------
        water = data.get("water_tank", {})
        if isinstance(water, dict):
            water_percent = water.get("level_percent", 0.0)
            water_liters = water.get("level_liters", 0.0)
            water_low = bool(water.get("low", False))
        else:
            water_percent = 0.0
            water_liters = 0.0
            water_low = False

        water_percent = max(0.0, min(100.0, water_percent))
        self.water_bar.setValue(int(water_percent))
        self.water_bar.setFormat(f"{water_percent:.1f}%")  # Show % inside bar
        self.water_bar.setTextVisible(True)

        # Water dot color
        if water_low:
            water_color = "#e74c3c"  # red
        else:
            water_color = "#2ecc71" if water_percent > 0 else "gray"
        self.water_dot.setStyleSheet(f"color: {water_color}; font-size: 18px;")

        # Water label above bar
        self.water_label.setText(f"Water: {water_liters:.1f} L")

        # ----------------- Sunlight -----------------
        sun = data.get("sunlight_ok", False)
        sun_ok = bool(sun)
        sun_color = "#2ecc71" if sun_ok else "#e74c3c"
        self.sun_dot.setStyleSheet(f"color: {sun_color}; font-size: 18px;")



# -----------------------------
# Other Screens (placeholders)
# -----------------------------
class RobotsScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)
        label = QLabel("Robots Screen")
        label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(label)

class MappingScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)
        label = QLabel("Mapping Screen")
        label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(label)

class ManualScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)
        label = QLabel("Manual Piloting Screen")
        label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(label)

# -----------------------------
# Dashboard Screen (buttons -> screens)
# -----------------------------
class DashboardScreen(BaseScreen):
    def __init__(self, telemetry_node, stacked_widget):
        super().__init__(stacked_widget)
        self.telemetry_node = telemetry_node

        grid = QGridLayout()
        self.layout.addLayout(grid)

        welcome_label = QLabel("Welcome to BloomBot!")
        welcome_label.setStyleSheet(Styles.HEADER)
        welcome_label.setAlignment(Qt.AlignTop)
        grid.addWidget(welcome_label, 0, 0)

        project_info = QLabel("Project info / description here")
        project_info.setWordWrap(True)
        project_info.setStyleSheet(Styles.LABEL)
        grid.addWidget(project_info, 0, 1)

        # Buttons
        buttons_layout = QVBoxLayout()
        self.robot_btn = QPushButton("Robots")
        self.live_btn = QPushButton("Live Monitoring")
        self.mapping_btn = QPushButton("Mapping")
        self.manual_btn = QPushButton("Manual Piloting")
        for btn in [self.robot_btn, self.live_btn, self.mapping_btn, self.manual_btn]:
            btn.setStyleSheet(Styles.BUTTON)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_layout.addWidget(btn)
        grid.addLayout(buttons_layout, 1, 0)

        placeholder = QLabel("More project info / placeholders")
        placeholder.setStyleSheet(Styles.LABEL)
        placeholder.setAlignment(Qt.AlignCenter)
        grid.addWidget(placeholder, 1, 1)

        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)

        # Screens
        self.live_monitoring_screen = LiveMonitoringScreen(telemetry_node, stacked_widget)
        self.robots_screen = RobotsScreen(stacked_widget)
        self.mapping_screen = MappingScreen(stacked_widget)
        self.manual_screen = ManualScreen(stacked_widget)

        for screen in [self.live_monitoring_screen, self.robots_screen,
                       self.mapping_screen, self.manual_screen]:
            self.stacked_widget.addWidget(screen)

        # Connect buttons
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
        "soil_moisture": {"raw": 240.0, "percent": 48.0},
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
