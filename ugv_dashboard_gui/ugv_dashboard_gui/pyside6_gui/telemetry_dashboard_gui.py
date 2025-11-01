# telemetry_dashboard_gui.py
import sys
import json
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PySide6.QtWidgets import (
    QApplication, QWidget, QStackedWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QGridLayout, QLineEdit, QSizePolicy,
    QProgressBar
)
from PySide6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np

from .gui_format import Styles

# -----------------------------
# Telemetry Listener Node
# -----------------------------
class TelemetryListener(Node):
    def __init__(self):
        super().__init__('telemetry_listener')
        self.telemetry_data = {}
        self.create_subscription(String, '/telemetry/snapshot', self.callback, 10)

    def callback(self, msg):
        try:
            self.telemetry_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON received')

    def update_data(self, data_dict):
        """Manually update telemetry data from terminal or other scripts."""
        self.telemetry_data = data_dict


# -----------------------------
# Radar Plot Widget
# -----------------------------
class RadarPlot(QWidget):
    def __init__(self, labels):
        super().__init__()
        self.labels = labels
        self.num_vars = len(labels)

        self.figure, self.ax = plt.subplots(figsize=(4,4), subplot_kw=dict(polar=True))
        self.canvas = FigureCanvas(self.figure)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def update(self, values):
        self.ax.clear()
        angles = np.linspace(0, 2 * np.pi, self.num_vars, endpoint=False).tolist()
        values = values + values[:1]
        angles += angles[:1]

        self.ax.set_theta_offset(np.pi / 2)
        self.ax.set_theta_direction(-1)
        self.ax.set_thetagrids(np.degrees(angles[:-1]), self.labels)
        self.ax.plot(angles, values, color='green', linewidth=2)
        self.ax.fill(angles, values, color='green', alpha=0.25)
        self.ax.set_ylim(0, 100)
        self.canvas.draw()


# -----------------------------
# Common Header Widget
# -----------------------------
class Header(QWidget):
    def __init__(self, stacked_widget, telemetry_node):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.telemetry_node = telemetry_node

        self.setStyleSheet(f"background-color: {Styles.BACKGROUND_COLOR};")
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        self.setLayout(layout)

        # Home & Logout
        self.home_btn = QPushButton("Home")
        self.home_btn.setStyleSheet(Styles.BUTTON)
        self.home_btn.clicked.connect(lambda: stacked_widget.setCurrentWidget(stacked_widget.dashboard_screen))

        self.logout_btn = QPushButton("Logout")
        self.logout_btn.setStyleSheet(Styles.BUTTON)
        self.logout_btn.clicked.connect(lambda: stacked_widget.setCurrentWidget(stacked_widget.login_screen))

        # Task status
        self.task_status_light = QLabel("●")
        self.task_status_light.setStyleSheet(f"color: {Styles.TASK_LIGHT_OFF}; font-size: 18px;")
        self.task_status_label = QLabel("Task: N/A")
        self.task_status_label.setStyleSheet(Styles.LABEL)

        layout.addWidget(self.home_btn)
        layout.addWidget(self.logout_btn)
        layout.addStretch()
        layout.addWidget(self.task_status_light)
        layout.addWidget(self.task_status_label)

        # Timer to update telemetry
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(2000)

    def update_telemetry(self):
        data = self.telemetry_node.telemetry_data
        task = data.get("task_status", "N/A")
        self.task_status_label.setText(f"Task: {task}")
        self.task_status_light.setStyleSheet(
            f"color: {Styles.TASK_COLOR if task != 'N/A' else Styles.TASK_LIGHT_OFF}; font-size: 18px;"
        )


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

        layout.addWidget(QLabel("Login to BloomBot", alignment=Qt.AlignCenter))
        layout.addWidget(self.username)
        layout.addWidget(self.password)
        layout.addWidget(self.login_btn)

        self.error_label = QLabel("", alignment=Qt.AlignCenter)
        self.error_label.setStyleSheet("color: red;")
        layout.addWidget(self.error_label)

        self.username.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.password.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.login_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def check_login(self):
        if self.username.text() == "bloombot" and self.password.text() == "robotics1!":
            self.switch_callback()
        else:
            self.error_label.setText("Invalid username or password")


# -----------------------------
# Base Screen class
# -----------------------------
class BaseScreen(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.setStyleSheet(f"background-color: {Styles.BACKGROUND_COLOR}; color: {Styles.TEXT_COLOR};")
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.header = Header(stacked_widget, stacked_widget.telemetry_node)
        self.layout.addWidget(self.header, alignment=Qt.AlignTop)


# -----------------------------
# Live Monitoring Screen
# -----------------------------
class LiveMonitoringScreen(BaseScreen):
    def __init__(self, telemetry_node, stacked_widget):
        super().__init__(stacked_widget)
        self.telemetry_node = telemetry_node

        left_layout = QVBoxLayout()
        self.sensors = {}

        for name in ["Water", "Soil", "Sunlight", "Seed", "Battery"]:
            label = QLabel(f"{name}: N/A")

            bar = QProgressBar()
            bar.setMaximum(100)
            bar.setFixedHeight(15)
            
            # Overlay label for percentage
            text_label = QLabel("0%", alignment=Qt.AlignCenter)
            text_label.setAttribute(Qt.WA_TransparentForMouseEvents)
            text_label.setStyleSheet(f"color: {Styles.SECONDARY_TEXT_COLOR}; background: transparent;")

            # Stack the progress bar and the overlay
            container = QStackedWidget()
            container.addWidget(bar)
            container.addWidget(text_label)

            left_layout.addWidget(label)
            left_layout.addWidget(container)

            self.sensors[name.lower()] = {
                "label": label,
                "bar": bar,
                "text_label": text_label
            }

        # Right side: radar plot
        self.radar_plot = RadarPlot(["Water", "Soil", "Sunlight", "Seed", "Battery"])

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout)
        main_layout.addWidget(self.radar_plot)

        self.layout.addLayout(main_layout)

        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(2000)

    def update_telemetry(self):
        data = self.header.telemetry_node.telemetry_data
        values_dict = {
            "water": data.get('water_tank', {}).get('level_percent', 0),
            "soil": data.get('soil_moisture', 0),
            "sunlight": 100 if data.get('sunlight_ok', 'N/A') == "Good" else 50,
            "seed": data.get('seed_level', 0),
            "battery": data.get('ugv_battery', 0)
        }

        def value_to_color(value):
            start_rgb = (14, 45, 17)
            end_rgb = (232, 255, 211)
            interp_rgb = tuple(int(s + (e - s) * value / 100) for s, e in zip(start_rgb, end_rgb))
            return "#{:02x}{:02x}{:02x}".format(*interp_rgb)

        # Store only valid values for the radar plot
        radar_values = []

        for name, val in values_dict.items():
            key = name.lower()
            bar = self.sensors[key]["bar"]
            label = self.sensors[key]["label"]

            # Check for invalid data
            if val > 100 or val < 0:
                bar.setValue(100)  # fill completely to show the error clearly
                bar.setStyleSheet("""
                    QProgressBar {
                        border: 1px solid #000;
                        border-radius: 5px;
                        text-align: center;
                        background-color: #2a2a2a;
                    }
                    QProgressBar::chunk {
                        background-color: #ff4d4d;  /* bright red */
                    }
                """)
                label.setText(f"{name.capitalize()}: Data Error")
                radar_values.append(None)  # mark invalid for radar plot
                continue

            # Normal range (0–100)
            bar.setValue(val)
            bar.setTextVisible(False)
            color = value_to_color(val)
            bar.setStyleSheet(f"""
                QProgressBar {{
                    border: 1px solid #000;
                    border-radius: 5px;
                    text-align: center;
                    background-color: #2a2a2a;
                }}
                QProgressBar::chunk {{
                    background-color: {color};
                }}
            """)
            label.setText(f"{name.capitalize()}: {val}%")
            radar_values.append(val)

        # Replace None values with 0 (so radar doesn’t draw them)
        cleaned_radar_values = [v if v is not None else 0 for v in radar_values]
        self.radar_plot.update(cleaned_radar_values)



class RobotsScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)
        label = QLabel("Robots Screen Placeholder")
        label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(label)

class MappingScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)
        label = QLabel("Mapping Screen Placeholder")
        label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(label)

class ManualScreen(BaseScreen):
    def __init__(self, stacked_widget):
        super().__init__(stacked_widget)
        label = QLabel("Manual Screen Placeholder")
        label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(label)


# -----------------------------
# Dashboard Screen
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
    thread = Thread(target=lambda: rclpy.spin(telemetry_node), daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    app.setStyleSheet(f"QWidget {{ background-color: {Styles.BACKGROUND_COLOR}; color: {Styles.TEXT_COLOR}; }}")
    main_window = BloomBotApp(telemetry_node)
    main_window.resize(900, 600)
    main_window.show()

    # Optional: hardcode some initial data
    telemetry_node.update_data({
        "task_status": "Idle",
        "water_tank": {"level_percent": 50},
        "soil_moisture": 30,
        "sunlight_ok": "Low",
        "seed_level": 100,
        "ugv_battery": 80
    })

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
