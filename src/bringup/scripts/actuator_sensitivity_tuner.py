#!/usr/bin/env python3
import sys
import os
import yaml
import math

from controller_manager_msgs.srv import ListHardwareInterfaces
import rclpy
from rclpy.node import Node

from PySide6.QtWidgets import (QMainWindow, QApplication,
                               QWidget, QSlider, QLabel,
                               QFormLayout, QVBoxLayout,
                               QHBoxLayout, QLineEdit,
                               QPushButton, QScrollArea, QStyle)
from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import QStyleOptionSlider
from PySide6.QtGui import QFont
from std_msgs.msg import Float64MultiArray
# Credit to: https://github.com/ros/joint_state_publisher/blob/ros2/joint_state_publisher_gui/joint_state_publisher_gui/joint_state_publisher_gui.py#L164
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200

class TuningSlider(QWidget):
    def __init__(self, name, min_val, max_val, initial_value=1.0, slider_max=100):
        super().__init__()

        self.min_val = min_val
        self.max_val = max_val
        self.slider_max = slider_max
        self.initial_value = initial_value

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        # Create a label
        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        # Create a disply box with the value of the slider
        self.display = QLineEdit("%.3f" % initial_value)
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout.addWidget(self.display)

        # Per-slider reset button
        self.reset_button = QPushButton("Reset")
        self.reset_button.setFixedHeight(22)
        self.reset_button.setFixedWidth(50)
        self.reset_button.clicked.connect(self.reset_value)
        self.row_layout.addWidget(self.reset_button)

        self.joint_layout.addLayout(self.row_layout)

        # Create a slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, slider_max)
        self.slider.setValue(self.value_to_slider(initial_value))
        self.slider.setFixedWidth(SLIDER_WIDTH)

        self.joint_layout.addWidget(self.slider)

        self._add_tick_labels()

        self.setLayout(self.joint_layout)

    def reset_value(self):
        self.slider.setValue(self.value_to_slider(self.initial_value))

    def value_to_slider(self, value):
        if value <= self.min_val:
            return 0
        t = math.log(value / self.min_val) / math.log(self.max_val / self.min_val)
        t = max(0.0, min(1.0, t))
        return int(round(t * self.slider_max))

    def slider_to_value(self, slider_value):
        if slider_value <= 0:
            return self.min_val
        t = slider_value / float(self.slider_max)
        return self.min_val * (self.max_val / self.min_val) ** t

    def _format_tick_label(self, exp):
        if exp == 0:
            return "1"
        return f"10<sup>{exp}</sup>"

    def _add_tick_labels(self):
        min_exp = math.ceil(math.log10(self.min_val))
        max_exp = math.floor(math.log10(self.max_val))
        if min_exp > max_exp:
            return

        tick_values = [10 ** exp for exp in range(min_exp, max_exp + 1)]
        tick_labels = [self._format_tick_label(exp) for exp in range(min_exp, max_exp + 1)]
        positions = [self.value_to_slider(v) for v in tick_values]

        self.tick_container = QWidget()
        self.tick_container.setFixedWidth(SLIDER_WIDTH)
        self.tick_container.setFixedHeight(14)

        self.tick_labels = []
        for pos, label in zip(positions, tick_labels):
            tick_label = QLabel(label, self.tick_container)
            tick_label.setFont(QFont("Helvetica", 8))
            tick_label.setTextFormat(Qt.RichText)
            tick_label.adjustSize()
            self.tick_labels.append((pos, tick_label))

        self.joint_layout.addWidget(self.tick_container, alignment=Qt.AlignLeft)
        self._position_tick_labels()

    def _position_tick_labels(self):
        if not hasattr(self, "tick_labels"):
            return
        opt = QStyleOptionSlider()
        self.slider.initStyleOption(opt)
        groove = self.slider.style().subControlRect(
            QStyle.CC_Slider, opt, QStyle.SC_SliderGroove, self.slider
        )
        handle = self.slider.style().subControlRect(
            QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self.slider
        )
        groove_x = groove.x()
        groove_w = groove.width()
        if groove_w <= 0:
            return
        for pos, label in self.tick_labels:
            t = pos / float(self.slider_max)
            x = int(groove_x + t * groove_w - label.width() / 2)
            # Keep labels within container bounds.
            x = max(0, min(x, self.tick_container.width() - label.width()))
            y = 0
            label.move(x, y)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_tick_labels()

    def remove(self):
        if hasattr(self, "tick_container"):
            self.joint_layout.removeWidget(self.tick_container)
            self.tick_container.setParent(None)

        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.reset_button)
        self.reset_button.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)

class ActuatorSensitivityTuner(Node, QMainWindow):
    sliderUpdateTrigger = Signal()
    initialize = Signal()
    def __init__(self):
        Node.__init__(self, "actuator_sensitivity_tuner")
        QMainWindow.__init__(self)

        self.declare_parameter("config_dir", "")
        config_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        if not config_dir:
            raise RuntimeError("config_dir parameter is required for actuator_sensitivity_tuner")
        os.makedirs(config_dir, exist_ok=True)
        self.save_file = os.path.join(config_dir, "actuator_sensitivity.yaml")

        self.cli = self.create_client(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces"
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for controller_manager...")

        self.claimed = []
        self.velocity_joints = {}
        self.position_joints = {}
        self.joint_order = []
        self.sensitivity_min = 1e-4
        self.sensitivity_max = 100.0
        self.slider_steps = 1000
        self.default_sensitivity = 1.0

        self.loaded_data = self.load_data()


        # refresh 5 times per second (adjust as needed)
        self.timer = self.create_timer(0.2, self.update_interfaces)
    

        self.joint_map = {}
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            "actuator_sensitivity",
            10
        )

        self.setWindowTitle("Actuator Sensitivity Tuner")
        self.resize(450, 700)

        # --- Main Layout ---
        self.main_layout = QVBoxLayout()

        # -- Button Layout --
        self.button_layout = QHBoxLayout()

        # Button to reset the sliders
        self.reset_button = QPushButton('Reset All Sensitivites', self)
        self.reset_button.clicked.connect(self.resetSliders)
        self.button_layout.addWidget(self.reset_button)
        
        self.main_layout.addLayout(self.button_layout)


        # -- Slider Layout -- 
        # Set up the scroll
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        # Widget inside scroll area
        self.scroll_widget = QWidget()

        self.slider_layout = QVBoxLayout()

        self.scroll_widget.setLayout(self.slider_layout)
        self.scroll_area.setWidget(self.scroll_widget)

        self.main_layout.addWidget(self.scroll_area)

        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.sliders = {}
        
        # Signal to initialize the window
        self.initialize.connect(self.initializeSliders)

        # Signal to update the sliders
        # (no position->slider sync needed for this GUI)

    # -- Data Persistence --
    def load_data(self):
        if not os.path.exists(self.save_file):
            return {}
        with open(self.save_file, "r") as f:
            data = yaml.safe_load(f)
        return data or {}

    def save_data(self):
        data = {}
        for joint_name, joint_info in self.joint_map.items():
            min_value = joint_info.get('min_sense', self.sensitivity_min)
            max_value = joint_info.get('max_sense', self.sensitivity_max)
            sensitivity = self.sliderToValue(
                float(joint_info['slider'].value()),
                min_val=min_value,
                max_val=max_value,
            )
            data[joint_name] = {
                "sensitivity": sensitivity,
                "min_value": min_value,
                "max_value": max_value
            }
        with open(self.save_file, "w") as f:
            yaml.safe_dump(data, f)


    # -- ROS2 Interface Update --
    def _done(self, future):
        result = future.result()
        claimed_changed = False

        new_claimed = [
            iface.name
            for iface in result.command_interfaces
            if iface.is_claimed
        ]

        if new_claimed != self.claimed:
            self.claimed = new_claimed
            claimed_changed = True
        
        self.velocity_joints = {
            claimed.split("/")[0]: claimed.split("/")[1]
            for claimed in self.claimed
            if "/velocity" in claimed
        }
        
        self.position_joints = {
            claimed.split("/")[0]: claimed.split("/")[1]
            for claimed in self.claimed
            if "/position" in claimed
        }

        if claimed_changed:
            # Rebuild sliders when claimed interfaces change.
            self.initialize.emit()

    def update_interfaces(self):
        req = ListHardwareInterfaces.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self._done)

    # -- GUI Management --
    def initializeSliders(self):
        self.joint_map = {}
        self.joint_order = []

        # Remove existing sliders
        for sl, _ in self.sliders.items():
            self.slider_layout.removeWidget(sl)
            sl.remove()

        # Generate sliders #
        for i, (joint_name, interface) in enumerate(self.velocity_joints.items()):
            joint_cfg = (self.loaded_data or {}).get(joint_name, {})
            min_val = joint_cfg.get("min_value", self.sensitivity_min)
            max_val = joint_cfg.get("max_value", self.sensitivity_max)
            sensitivity = joint_cfg.get("sensitivity", self.default_sensitivity)

            slider = TuningSlider(
                joint_name,
                min_val,
                max_val,
                sensitivity,
                slider_max=self.slider_steps,
            )

            self.joint_map[joint_name] = {
                'display': slider.display, 
                'slider': slider.slider,
                'min_sense': min_val,
                'max_sense': max_val,
            }
            
            self.joint_order.append(joint_name)

            self.slider_layout.addWidget(slider)
            # Connect to the signal provided by QSignal
            slider.slider.valueChanged.connect(
                lambda event, name=joint_name: self.onSliderValueChangedOne(name)
            )

            self.sliders[slider] = slider

        # Set initial sensitivity values read from parameters
        # self.resetSliders()

        self.publish_sensitivities()

    def onSliderValueChangedOne(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]
        slidervalue = joint_info['slider'].value()
        joint_sensitivity = self.sliderToValue(
            slidervalue,
            min_val=joint_info.get('min_sense', self.sensitivity_min),
            max_val=joint_info.get('max_sense', self.sensitivity_max),
        )
        joint_info['display'].setText("%.3f" % joint_sensitivity)
        self.publish_sensitivities()
    
    def sliderToValue(self, slider, max_value=None, min_val=None, max_val=None):
        # Logarithmic mapping: slider [0..max_value] -> [min_val..max_val]
        if max_value is None:
            max_value = self.slider_steps
        if min_val is None:
            min_val = self.sensitivity_min
        if max_val is None:
            max_val = self.sensitivity_max
        if slider <= 0:
            return min_val
        t = slider / float(max_value)
        return min_val * (max_val / min_val) ** t

    def valueToSlider(self, value, max_value=None, min_val=None, max_val=None):
        # Inverse of sliderToValue
        if max_value is None:
            max_value = self.slider_steps
        if min_val is None:
            min_val = self.sensitivity_min
        if max_val is None:
            max_val = self.sensitivity_max
        if value <= min_val:
            return 0
        t = math.log(value / min_val) / math.log(max_val / min_val)
        t = max(0.0, min(1.0, t))
        return int(round(t * max_value))

    def resetSliders(self):
        default_slider = self.valueToSlider(self.default_sensitivity)
        for joint_info in self.joint_map.values():
            joint_info['slider'].setValue(default_slider)
            joint_info['display'].setText("%.3f" % self.default_sensitivity)
        self.publish_sensitivities()

    def collect_sensitivities(self):
        sensitivities = []
        for name in self.joint_order:
            joint_info = self.joint_map.get(name)
            if joint_info is None:
                continue
            sensitivities.append(
                self.sliderToValue(
                    float(joint_info['slider'].value()),
                    min_val=joint_info.get('min_sense', self.sensitivity_min),
                    max_val=joint_info.get('max_sense', self.sensitivity_max),
                )
            )
        return sensitivities

    def publish_sensitivities(self):
        msg = Float64MultiArray()
        msg.data = self.collect_sensitivities()
        self.publisher_.publish(msg)
        self.save_data()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = ActuatorSensitivityTuner()

    # Pump ROS from Qt
    timer = QTimer()
    def spin_once():
        if not rclpy.ok():
            timer.stop()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            # Avoid crashing if shutdown happens while Qt is still ticking.
            timer.stop()
    timer.timeout.connect(spin_once)
    timer.start(10)  # ms

    node.show()
    exit_code = app.exec()
    if timer.isActive():
        timer.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)



if __name__ == "__main__":
    main()
