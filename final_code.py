import time
import math
import json
import os
import threading
import queue
import urllib.request
from collections import deque
import sys

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
import jkrc

ROBOT_IP = "10.5.5.100"
CONFIG_FILE = "spatial_model.json"
J2_COLLISION_THRESHOLD = 0.0  # J2 at or below this value = collision detected

BOLT_PROFILES = {
    "M4": 5.0, "M5": 6.0, "M6": 10.0, "M8": 15.0, "M10": 20.0,
    "M12": 25.0, "M14": 28.0, "M16": 30.0, "M18": 30.0, "M20": 30.0
}

AREG_DASHBOARD_STYLE = """
    QWidget {
        background-color: #08090B;
        color: #E8EAF0;
        font-family: 'Segoe UI', 'Inter', system-ui, sans-serif;
    }

    /* ── Header bar ───────────────────────────────────────── */
    QFrame#HeaderBar {
        background-color: #0C0D10;
        border-bottom: 1px solid #1A1D27;
        border-radius: 0px;
    }
    QLabel#AppTitle {
        font-size: 15px; font-weight: 700; color: #F1F5F9;
        letter-spacing: 0.3px;
    }
    QLabel#AppSubtitle {
        font-size: 11px; font-weight: 400; color: #4B5563;
        letter-spacing: 0.5px;
    }
    QFrame#LogoSeparator {
        background-color: #1F2937;
        max-width: 1px; min-width: 1px;
        margin-top: 6px; margin-bottom: 6px;
    }

    /* ── Cards ────────────────────────────────────────────── */
    QFrame#DashboardCard {
        background-color: #0F1117;
        border: 1px solid #1A1D27;
        border-radius: 10px;
    }
    QLabel#CardTitle {
        font-size: 10px; font-weight: 700; color: #4B5563;
        letter-spacing: 1.2px;
    }

    /* ── Telemetry text ───────────────────────────────────── */
    QLabel#TelemetryText {
        font-family: 'Consolas', 'Courier New', monospace;
        font-size: 14px; color: #34D399;
        background-color: #050608; padding: 10px; border-radius: 6px;
        border: 1px solid #1A1D27;
    }

    /* ── Alert labels ─────────────────────────────────────── */
    QLabel#AlertNone  { color: #374151; font-size: 13px; font-weight: 600; }
    QLabel#AlertActive { color: #EF4444; font-size: 13px; font-weight: 700; }

    /* ── Status dot ───────────────────────────────────────── */
    QLabel#StatusDot { font-size: 10px; }

    /* ── Buttons (base) ───────────────────────────────────── */
    QPushButton {
        border-radius: 7px; font-weight: 600; font-size: 13px;
        padding: 8px 14px; border: none;
    }

    QPushButton#ConnectBtn {
        background-color: #10B981; color: #05070A;
        font-size: 12px; font-weight: 700; padding: 9px 16px;
        border-radius: 7px;
    }
    QPushButton#ConnectBtn:hover { background-color: #059669; }
    QPushButton#ConnectBtn:disabled { background-color: #1A1D27; color: #374151; }

    QPushButton#PrimaryAction {
        background-color: #F15A29; color: #FFFFFF;
        font-size: 15px; font-weight: 700; padding: 14px;
        border-radius: 8px;
    }
    QPushButton#PrimaryAction:hover { background-color: #D4431B; }
    QPushButton#PrimaryAction:disabled { background-color: #1F2937; color: #374151; }

    QPushButton#DangerAction {
        background-color: #EF4444; color: #FFFFFF;
        font-size: 13px; font-weight: 700; padding: 14px;
        border-radius: 8px;
    }
    QPushButton#DangerAction:hover { background-color: #DC2626; }

    QPushButton#SecondaryBtn {
        background-color: #141720; color: #CBD5E1;
        border: 1px solid #1F2937; border-radius: 7px;
        font-size: 12px;
    }
    QPushButton#SecondaryBtn:hover { background-color: #1A1D2E; border-color: #374151; }

    /* ── Dropdowns / spinboxes ────────────────────────────── */
    QComboBox, QSpinBox {
        background-color: #141720; color: #E8EAF0;
        border: 1px solid #1F2937; border-radius: 6px;
        padding: 6px 10px; font-size: 12px;
    }
    QComboBox:hover, QSpinBox:hover { border-color: #F15A29; }
    QComboBox::drop-down { border: none; }

    /* ── Slider ───────────────────────────────────────────── */
    QSlider::groove:horizontal {
        height: 4px; background: #1F2937; border-radius: 2px;
    }
    QSlider::handle:horizontal {
        background: #F15A29; width: 14px; height: 14px;
        margin-top: -5px; margin-bottom: -5px;
        border-radius: 7px;
    }
    QSlider::sub-page:horizontal {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #F15A29, stop:1 #F7941D);
        border-radius: 2px;
    }

    /* ── Console ──────────────────────────────────────────── */
    QTextEdit#Console {
        background-color: #050608; color: #34D399;
        font-family: 'Consolas', monospace; font-size: 11px;
        border: 1px solid #1A1D27; border-radius: 8px; padding: 8px;
    }

    /* ── Jog buttons (position) ───────────────────────────── */
    QPushButton#JogBtn {
        background-color: #0F1117; color: #4B5563;
        border: 1px solid #1A1D27; border-radius: 10px;
        font-size: 16px; font-weight: 800;
        min-width: 54px; max-width: 54px;
        min-height: 54px; max-height: 54px;
    }
    QPushButton#JogBtn:hover {
        background-color: #1C1F2E; color: #F15A29;
        border: 1px solid #F15A29;
    }
    QPushButton#JogBtn:pressed {
        background-color: #F15A29; color: #FFFFFF;
        border: 1px solid #D4431B;
    }
"""

class DashboardCard(QtWidgets.QFrame):
    """Custom UI component to create the sleek dashboard aesthetic."""
    def __init__(self, title_text=""):
        super().__init__()
        self.setObjectName("DashboardCard")
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(15, 15, 15, 15)
        self.layout.setSpacing(12)
        
        if title_text:
            title = QtWidgets.QLabel(title_text)
            title.setObjectName("CardTitle")
            self.layout.addWidget(title)

class BoltTighteningApp(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Areg.AI | Autonomous Bolt Tightening Dashboard")
        self.resize(1300, 900)
        self.setMinimumSize(900, 650)
        self.setStyleSheet(AREG_DASHBOARD_STYLE)

        self.robot = jkrc.RC(ROBOT_IP)
        self.is_connected = False
        self.abort_flag = False
        
        # --- High-Speed Telemetry Variables ---
        self.telemetry_running = False
        self.latest_torques = [0.0] * 6
        
        MAX_LIVE_POINTS = 300
        self.time_steps = deque(maxlen=MAX_LIVE_POINTS)
        self.torque_history = [deque(maxlen=MAX_LIVE_POINTS) for _ in range(6)]
        
        self.touch_detected = False
        self._use_fast_torque = False
        self.auto_scroll = True
        self.start_time = 0.0
        
        self.log_queue = queue.Queue()
        self.hover_poses = []
        self.target_bolt_count = 1
        self.jog_speed = 80.0   # mm/s for position jog
        
        self.model = self.load_model()
        self._logo_pixmap = self._load_logo()
        self.build_ui()
        
        # 100Hz Master UI Update Timer (10ms)
        self.ui_timer = QtCore.QTimer()
        self.ui_timer.timeout.connect(self.update_live_ui)
        self.ui_timer.start(10)
        
        self.monitor_timer = QtCore.QTimer()
        self.monitor_timer.timeout.connect(self.monitor_connection)
        self.monitor_timer.start(2000)


    def _load_logo(self):
        local = os.path.join(os.path.dirname(os.path.abspath(__file__)), "areg_logo.png")
        if not os.path.exists(local):
            try:
                url = "https://areg.ai/wp-content/uploads/2025/11/icon_logo-main.png"
                urllib.request.urlretrieve(url, local)
            except Exception:
                return None
        px = QtGui.QPixmap(local)
        if px.isNull():
            return None
        return px.scaledToHeight(38, QtCore.Qt.SmoothTransformation)

    def load_model(self):
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
                if "saved_waypoints" not in data:
                    data["saved_waypoints"] = {}
                return data
        return {"bolt_type": "M10", "torque_limit": 20.0, "saved_waypoints": {}}

    def save_model(self):
        with open(CONFIG_FILE, 'w') as f:
            json.dump(self.model, f, indent=4)

    def log(self, message):
        self.log_queue.put(message)

    def build_ui(self):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(15)

        # --- HEADER BAR ---
        header_bar = QtWidgets.QFrame()
        header_bar.setObjectName("HeaderBar")
        header_bar.setFixedHeight(62)
        header_h = QtWidgets.QHBoxLayout(header_bar)
        header_h.setContentsMargins(16, 0, 16, 0)
        header_h.setSpacing(14)

        # Logo image (or text fallback)
        if self._logo_pixmap:
            logo_lbl = QtWidgets.QLabel()
            logo_lbl.setPixmap(self._logo_pixmap)
            logo_lbl.setFixedSize(self._logo_pixmap.width(), self._logo_pixmap.height())
            header_h.addWidget(logo_lbl)
        else:
            logo_text = QtWidgets.QLabel("Areg<span style='color:#F15A29;'>AI</span>")
            logo_text.setTextFormat(QtCore.Qt.RichText)
            logo_text.setStyleSheet(
                "font-size: 20px; font-weight: 800; color: #F1F5F9; letter-spacing: -0.5px;"
            )
            header_h.addWidget(logo_text)

        # Vertical separator
        vsep = QtWidgets.QFrame()
        vsep.setObjectName("LogoSeparator")
        vsep.setFrameShape(QtWidgets.QFrame.VLine)
        header_h.addWidget(vsep)

        # Title + subtitle stack
        title_col = QtWidgets.QVBoxLayout()
        title_col.setSpacing(1)
        app_title_lbl = QtWidgets.QLabel("Autonomous Bolt Tightening")
        app_title_lbl.setObjectName("AppTitle")
        app_sub_lbl = QtWidgets.QLabel("ROBOTIC FLEET CONTROL  ·  JAKA COBOT")
        app_sub_lbl.setObjectName("AppSubtitle")
        title_col.addWidget(app_title_lbl)
        title_col.addWidget(app_sub_lbl)
        header_h.addLayout(title_col)

        header_h.addStretch()

        # Status indicator
        self.status_dot = QtWidgets.QLabel("● OFFLINE")
        self.status_dot.setObjectName("StatusDot")
        self.status_dot.setStyleSheet("color: #374151; font-size: 11px; font-weight: 700; letter-spacing: 0.5px;")
        header_h.addWidget(self.status_dot)

        self.btn_connect = QtWidgets.QPushButton("CONNECT ROBOT")
        self.btn_connect.setObjectName("ConnectBtn")
        self.btn_connect.setFixedWidth(160)
        self.btn_connect.clicked.connect(self.connect_robot)
        header_h.addWidget(self.btn_connect)

        main_layout.addWidget(header_bar)

        # --- MAIN CONTENT (padded wrapper) ---
        content_wrapper = QtWidgets.QWidget()
        content_wrapper.setContentsMargins(0, 0, 0, 0)
        wrapper_layout = QtWidgets.QVBoxLayout(content_wrapper)
        wrapper_layout.setContentsMargins(18, 14, 18, 14)
        wrapper_layout.setSpacing(0)
        main_layout.addWidget(content_wrapper)

        content_layout = QtWidgets.QHBoxLayout()

        # --- LEFT COLUMN (CONTROLS) ---
        left_column = QtWidgets.QVBoxLayout()
        left_column.setSpacing(12)
        
        # Card 1: Configuration
        config_card = DashboardCard("System Configuration")
        
        config_card.layout.addWidget(QtWidgets.QLabel("Target Bolt Profile:"))
        self.bolt_dropdown = QtWidgets.QComboBox()
        self.bolt_dropdown.addItems(list(BOLT_PROFILES.keys()))
        self.bolt_dropdown.setCurrentText(self.model["bolt_type"])
        self.bolt_dropdown.currentTextChanged.connect(self.change_bolt)
        config_card.layout.addWidget(self.bolt_dropdown)
        
        config_card.layout.addWidget(QtWidgets.QLabel("Sequence Targets (1-5):"))
        self.bolt_count_dropdown = QtWidgets.QComboBox()
        self.bolt_count_dropdown.addItems(["1", "2", "3", "4", "5"])
        self.bolt_count_dropdown.setCurrentText("1")
        self.bolt_count_dropdown.currentTextChanged.connect(self.change_bolt_count)
        config_card.layout.addWidget(self.bolt_count_dropdown)
        left_column.addWidget(config_card)

        # Card 2: Digital Twin Training
        teach_card = DashboardCard("Digital Twin Mapping")
        self.lbl_teach_status = QtWidgets.QLabel("Waypoints: 0 / 1")
        self.lbl_teach_status.setStyleSheet("color: #F15A29; font-size: 14px; font-weight: bold;")
        teach_card.layout.addWidget(self.lbl_teach_status)
        
        self.btn_teach_hover = QtWidgets.QPushButton("TEACH WAYPOINT")
        self.btn_teach_hover.setObjectName("SecondaryBtn")
        self.btn_teach_hover.clicked.connect(self.teach_hover)
        teach_card.layout.addWidget(self.btn_teach_hover)
        
        self.btn_clear_poses = QtWidgets.QPushButton("WIPE MEMORY")
        self.btn_clear_poses.setStyleSheet("background-color: transparent; border: 1px solid #EF4444; color: #EF4444;")
        self.btn_clear_poses.clicked.connect(self.clear_poses)
        teach_card.layout.addWidget(self.btn_clear_poses)
        left_column.addWidget(teach_card)

        # Card 3: Profile Management
        mem_card = DashboardCard("Profile Library")
        self.saved_profiles_dropdown = QtWidgets.QComboBox()
        mem_card.layout.addWidget(self.saved_profiles_dropdown)
        self.update_profiles_dropdown()

        mem_btn_layout = QtWidgets.QHBoxLayout()
        self.btn_load_profile = QtWidgets.QPushButton("LOAD")
        self.btn_load_profile.setObjectName("SecondaryBtn")
        self.btn_load_profile.clicked.connect(self.load_selected_waypoints)
        mem_btn_layout.addWidget(self.btn_load_profile)
        
        self.btn_save_profile = QtWidgets.QPushButton("SAVE")
        self.btn_save_profile.setObjectName("SecondaryBtn")
        self.btn_save_profile.clicked.connect(self.save_current_waypoints)
        mem_btn_layout.addWidget(self.btn_save_profile)
        mem_card.layout.addLayout(mem_btn_layout)
        left_column.addWidget(mem_card)

        # Card 4: Manual Jog Control
        jog_card = DashboardCard("Manual Jog Control")

        # ── Speed slider ─────────────────────────────────────
        speed_row = QtWidgets.QHBoxLayout()
        spd_lbl = QtWidgets.QLabel("Speed:")
        spd_lbl.setStyleSheet("color: #6B7280; font-size: 12px; min-width: 46px;")
        speed_row.addWidget(spd_lbl)
        self.jog_speed_val_lbl = QtWidgets.QLabel("80 mm/s")
        self.jog_speed_val_lbl.setStyleSheet(
            "color: #F15A29; font-size: 12px; font-weight: bold; min-width: 62px;"
        )
        self.jog_speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.jog_speed_slider.setRange(5, 250)
        self.jog_speed_slider.setValue(80)
        self.jog_speed_slider.valueChanged.connect(self._update_jog_speed)
        speed_row.addWidget(self.jog_speed_slider)
        speed_row.addWidget(self.jog_speed_val_lbl)
        jog_card.layout.addLayout(speed_row)

        # ── Position jog: Z pill + XY cross (hold to move) ───
        controls_row = QtWidgets.QHBoxLayout()
        controls_row.setSpacing(18)
        controls_row.setAlignment(QtCore.Qt.AlignCenter)

        z_col = QtWidgets.QVBoxLayout()
        z_col.setSpacing(4)
        z_col.setAlignment(QtCore.Qt.AlignCenter)
        btn_zp = QtWidgets.QPushButton("▲\nZ+")
        btn_zp.setObjectName("JogBtn")
        btn_zp.pressed.connect(lambda: self._start_pos_jog('z', +1))
        btn_zp.released.connect(self._stop_pos_jog)
        z_col.addWidget(btn_zp, alignment=QtCore.Qt.AlignCenter)
        z_lbl = QtWidgets.QLabel("Z")
        z_lbl.setAlignment(QtCore.Qt.AlignCenter)
        z_lbl.setStyleSheet("color: #374151; font-size: 10px; font-weight: bold; letter-spacing: 2px;")
        z_col.addWidget(z_lbl)
        btn_zm = QtWidgets.QPushButton("Z−\n▼")
        btn_zm.setObjectName("JogBtn")
        btn_zm.pressed.connect(lambda: self._start_pos_jog('z', -1))
        btn_zm.released.connect(self._stop_pos_jog)
        z_col.addWidget(btn_zm, alignment=QtCore.Qt.AlignCenter)
        controls_row.addLayout(z_col)

        vsep = QtWidgets.QFrame()
        vsep.setFrameShape(QtWidgets.QFrame.VLine)
        vsep.setStyleSheet("color: #1F2937;")
        controls_row.addWidget(vsep)

        xy_grid = QtWidgets.QGridLayout()
        xy_grid.setSpacing(4)
        btn_yp = QtWidgets.QPushButton("▲\nY+")
        btn_yp.setObjectName("JogBtn")
        btn_yp.pressed.connect(lambda: self._start_pos_jog('y', +1))
        btn_yp.released.connect(self._stop_pos_jog)
        xy_grid.addWidget(btn_yp, 0, 1, alignment=QtCore.Qt.AlignCenter)
        btn_xm = QtWidgets.QPushButton("◄\nX−")
        btn_xm.setObjectName("JogBtn")
        btn_xm.pressed.connect(lambda: self._start_pos_jog('x', -1))
        btn_xm.released.connect(self._stop_pos_jog)
        xy_grid.addWidget(btn_xm, 1, 0, alignment=QtCore.Qt.AlignCenter)
        xy_ctr = QtWidgets.QLabel("⊕")
        xy_ctr.setAlignment(QtCore.Qt.AlignCenter)
        xy_ctr.setFixedSize(54, 54) 
        xy_ctr.setStyleSheet(
            "color: #1F2937; font-size: 26px; border: 1px solid #1F2937;"
            " border-radius: 10px; background-color: #0D1117;"
        )
        xy_grid.addWidget(xy_ctr, 1, 1, alignment=QtCore.Qt.AlignCenter)
        btn_xp = QtWidgets.QPushButton("X+\n►")
        btn_xp.setObjectName("JogBtn")
        btn_xp.pressed.connect(lambda: self._start_pos_jog('x', +1))
        btn_xp.released.connect(self._stop_pos_jog)
        xy_grid.addWidget(btn_xp, 1, 2, alignment=QtCore.Qt.AlignCenter)
        btn_ym = QtWidgets.QPushButton("Y−\n▼")
        btn_ym.setObjectName("JogBtn")
        btn_ym.pressed.connect(lambda: self._start_pos_jog('y', -1))
        btn_ym.released.connect(self._stop_pos_jog)
        xy_grid.addWidget(btn_ym, 2, 1, alignment=QtCore.Qt.AlignCenter)
        controls_row.addLayout(xy_grid)
        jog_card.layout.addLayout(controls_row)

        # ── Fixed orientation button ──────────────────────────
        hsep = QtWidgets.QFrame()
        hsep.setFrameShape(QtWidgets.QFrame.HLine)
        hsep.setStyleSheet("color: #1F2937;")
        jog_card.layout.addWidget(hsep)

        btn_fix_ori = QtWidgets.QPushButton("Set Orientation  Rx:180°  Ry:0°  Rz:180°")
        btn_fix_ori.setStyleSheet("""
            QPushButton {
                background-color: #1E1B48; color: #A78BFA;
                border: 1px solid #4C1D95; border-radius: 6px;
                font-size: 11px; font-weight: 700;
                padding: 8px 12px;
            }
            QPushButton:hover { background-color: #2D1B69; border-color: #7C3AED; color: #DDD6FE; }
            QPushButton:pressed { background-color: #7C3AED; color: #FFFFFF; }
        """)
        btn_fix_ori.clicked.connect(self._set_fixed_orientation)
        jog_card.layout.addWidget(btn_fix_ori)

        left_column.addWidget(jog_card)

        left_column.addStretch()
        content_layout.addLayout(left_column, stretch=1)

        # --- RIGHT COLUMN (TELEMETRY & EXECUTION) ---
        right_column = QtWidgets.QVBoxLayout()
        right_column.setSpacing(15)

        # Execution Header
        exec_layout = QtWidgets.QHBoxLayout()
        self.btn_run = QtWidgets.QPushButton("▶ INITIATE SMART SEQUENCE")
        self.btn_run.setObjectName("PrimaryAction")
        self.btn_run.clicked.connect(self.run_sequence)
        exec_layout.addWidget(self.btn_run, stretch=2)

        self.btn_estop = QtWidgets.QPushButton("EMERGENCY OVERRIDE")
        self.btn_estop.setObjectName("DangerAction")
        self.btn_estop.clicked.connect(self.emergency_stop)
        exec_layout.addWidget(self.btn_estop, stretch=1)
        right_column.addLayout(exec_layout)

        # Card 4: Telemetry Feed
        telem_card = DashboardCard("Live Telemetry & Detection")
        self.torque_label = QtWidgets.QLabel("STATUS: OFFLINE")
        self.torque_label.setObjectName("TelemetryText")
        telem_card.layout.addWidget(self.torque_label)
        
        alert_layout = QtWidgets.QHBoxLayout()
        alert_layout.addWidget(QtWidgets.QLabel("COLLISION SENSOR:"))
        self.alert_label = QtWidgets.QLabel("STANDBY")
        self.alert_label.setObjectName("AlertNone")
        alert_layout.addWidget(self.alert_label)
        alert_layout.addStretch()
        telem_card.layout.addLayout(alert_layout)
        right_column.addWidget(telem_card)

        # Card 5: Live Graph
        graph_card = DashboardCard("Joint Torque Oscilloscope")
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#111827') # Matches card background
        self.plot_widget.showGrid(x=True, y=True, alpha=0.15)
        self.plot_widget.setLabel('bottom', 'Time (s)', color='#9CA3AF')
        self.plot_widget.setLabel('left', 'Torque (Nm)', color='#9CA3AF')
        
        # Style the legend
        legend = self.plot_widget.addLegend()
        legend.setLabelTextColor('#E2E8F0')
        
        graph_card.layout.addWidget(self.plot_widget)
        
        self.btn_scroll = QtWidgets.QPushButton("Pause Auto-Scroll")
        self.btn_scroll.setObjectName("SecondaryBtn")
        self.btn_scroll.clicked.connect(self.toggle_scroll)
        graph_card.layout.addWidget(self.btn_scroll)

        joint_names = ["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)", "J4 (Wrist 1)", "J5 (Wrist 2)", "J6 (Wrist 3)"]
        colors = ['#EF4444', '#10B981', '#F59E0B', '#3B82F6', '#EC4899', '#8B5CF6']
        self.curves = []
        for i in range(6):
            curve = self.plot_widget.plot(pen=pg.mkPen(colors[i], width=2), name=joint_names[i])
            self.curves.append(curve)
            
        right_column.addWidget(graph_card, stretch=2)

        # Console Output
        self.console_text = QtWidgets.QTextEdit()
        self.console_text.setReadOnly(True)
        self.console_text.setObjectName("Console")
        right_column.addWidget(self.console_text, stretch=1)

        content_layout.addLayout(right_column, stretch=3)
        wrapper_layout.addLayout(content_layout)

        self.log("Areg.AI Core Initialized. Dashboard Ready. Awaiting Fleet Connection...")

    # --- TELEMETRY BACKGROUND THREAD ---
    def _telemetry_worker(self):
        self.start_time = time.time()
        self.log("[SYS] Telemetry active — reading joint torques via get_robot_status().")

        while self.telemetry_running and self.is_connected:
            try:
                res = self.robot.get_robot_status()
                if isinstance(res, (list, tuple)) and res[0] == 0:
                    self.latest_torques = [float(res[1][20][5][i][4]) for i in range(6)]

                current_time = time.time() - self.start_time
                self.time_steps.append(current_time)
                for i in range(6):
                    self.torque_history[i].append(self.latest_torques[i])

                # Collision = J2 (Shoulder) drops to zero or below.
                # Under normal descent J2 carries the arm weight and stays
                # well above zero; zero means the load reversed (hit something).
                if self.latest_torques[1] <= J2_COLLISION_THRESHOLD:
                    self.touch_detected = True

            except Exception:
                pass

            time.sleep(0.005)

    # --- MASTER UI UPDATER (100Hz) ---
    def update_live_ui(self):
        # 1. Flush console logs
        while not self.log_queue.empty():
            msg = self.log_queue.get()
            self.console_text.append(msg)
            scrollbar = self.console_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

        # 2. Update Graph securely to prevent array-mismatch crashes
        snap_x = list(self.time_steps)
        snap_y = [list(th) for th in self.torque_history]
        min_len = min([len(snap_x)] + [len(y) for y in snap_y])

        if min_len >= 2:
            safe_x = snap_x[:min_len]
            for i in range(6):
                self.curves[i].setData(safe_x, snap_y[i][:min_len])

            if self.auto_scroll:
                current_time = safe_x[-1]
                self.plot_widget.setXRange(max(0, current_time - 3.0), current_time + 0.2, padding=0)

                recent_vals = [val for y in snap_y for val in y]
                if recent_vals:
                    self.plot_widget.setYRange(min(recent_vals) - 5, max(recent_vals) + 5, padding=0)

        # 3. Update Text Dashboard
        if self.is_connected:
            t = self.latest_torques
            t_str = f"J1: {t[0]:.1f} | J2: {t[1]:.1f} | J3: {t[2]:.1f} | J4: {t[3]:.1f} | J5: {t[4]:.1f} | J6: {t[5]:.1f}"
            self.torque_label.setText(t_str)

            if self.touch_detected:
                self.alert_label.setObjectName("AlertActive")
                self.alert_label.setText(f"J2 COLLISION DETECTED  ({self.latest_torques[1]:.1f} Nm ≤ 0)")
            else:
                self.alert_label.setObjectName("AlertNone")
                self.alert_label.setText("CLEAR")

            # Refresh stylesheet for dynamic ID changes
            self.alert_label.style().unpolish(self.alert_label)
            self.alert_label.style().polish(self.alert_label)
        else:
            self.torque_label.setText("STATUS: OFFLINE")

    def toggle_scroll(self):
        self.auto_scroll = not self.auto_scroll
        if self.auto_scroll:
            self.btn_scroll.setText('Pause Auto-Scroll')
        else:
            self.btn_scroll.setText('Resume Auto-Scroll')

    def connect_robot(self):
        self.btn_connect.setEnabled(False)
        self.btn_connect.setText("ESTABLISHING LINK...")
        def task():
            self.log("[SYS] Establishing secure handshake with robot...")
            try:
                self.robot.login("Administrator", "jakazuadmin")
                self.robot.power_on()
                self.robot.enable_robot()
                time.sleep(1.5)
                
                status_res = self.robot.get_robot_status()
                if status_res and status_res[0] == 0:
                    self.is_connected = True
                    self.telemetry_running = True
                    threading.Thread(target=self._telemetry_worker, daemon=True).start()
                    
                    self.btn_connect.setText("CONNECTED")
                    self.btn_connect.setObjectName("SecondaryBtn")
                    self.btn_connect.style().unpolish(self.btn_connect)
                    self.btn_connect.style().polish(self.btn_connect)
                    self.status_dot.setText("● ONLINE")
                    self.status_dot.setStyleSheet("color: #10B981; font-size: 11px; font-weight: 700; letter-spacing: 0.5px;")
                    self.log("[SYS] Secure connection established. High-speed telemetry active.")
                else:
                    raise Exception("Initialization sequence timed out.")
            except Exception as e:
                self.handle_disconnect(str(e))
        threading.Thread(target=task, daemon=True).start()

    def monitor_connection(self):
        if self.is_connected:
            try:
                if hasattr(self.robot, 'get_robot_status_simple'):
                    res = self.robot.get_robot_status_simple()
                else:
                    res = self.robot.get_robot_status()
                if not res or res[0] != 0:
                    self.handle_disconnect("Robot state transitioned to offline.")
            except Exception as e:
                self.handle_disconnect(f"Link lost: {e}")

    def handle_disconnect(self, reason):
        self.is_connected = False
        self.telemetry_running = False
        self.btn_connect.setEnabled(True)
        self.btn_connect.setText("CONNECT ROBOT")
        self.btn_connect.setObjectName("ConnectBtn")
        self.btn_connect.style().unpolish(self.btn_connect)
        self.btn_connect.style().polish(self.btn_connect)
        self.status_dot.setText("● OFFLINE")
        self.status_dot.setStyleSheet("color: #374151; font-size: 11px; font-weight: 700; letter-spacing: 0.5px;")
        self.log(f"[ERR] Fleet Offline: {reason}")
        if not self.abort_flag:
            self.emergency_stop()

    def change_bolt(self, choice):
        self.model["bolt_type"] = choice
        self.model["torque_limit"] = BOLT_PROFILES[choice]
        self.save_model()
        self.log(f"[DATA] Component profile updated to {choice}.")

    def change_bolt_count(self, choice):
        self.target_bolt_count = int(choice)
        self.clear_poses()
        self.lbl_teach_status.setText(f"Waypoints: {len(self.hover_poses)} / {self.target_bolt_count}")
        self.log(f"[DATA] Sequence mapped for {choice} targets.")

    def clear_poses(self):
        self.hover_poses.clear()
        self.lbl_teach_status.setStyleSheet("color: #F15A29; font-size: 14px; font-weight: bold;")
        self.lbl_teach_status.setText(f"Waypoints: 0 / {self.target_bolt_count}")
        self.log("[SYS] Current spatial memory wiped.")

    def teach_hover(self):
        if not self.is_connected:
            self.log("[ERR] Robot offline.")
            return
        if len(self.hover_poses) >= self.target_bolt_count:
            self.log("[SYS] Sequence capacity reached.")
            return

        res_cart = self.robot.get_tcp_position()
        res_joint = self.robot.get_joint_position()
        
        if res_cart[0] == 0 and res_joint[0] == 0:
            pose_data = {"cartesian": list(res_cart[1]), "joints": list(res_joint[1])}
            self.hover_poses.append(pose_data)
            
            taught_count = len(self.hover_poses)
            self.log(f"[MEM] Waypoint {taught_count} registered in Digital Twin.")
            
            if taught_count >= self.target_bolt_count:
                self.lbl_teach_status.setStyleSheet("color: #10B981; font-size: 14px; font-weight: bold;")
                self.lbl_teach_status.setText(f"Waypoints: {taught_count} / {self.target_bolt_count} - DEPLOY READY")
            else:
                self.lbl_teach_status.setStyleSheet("color: #F15A29; font-size: 14px; font-weight: bold;")
                self.lbl_teach_status.setText(f"Waypoints: {taught_count} / {self.target_bolt_count}")

    def update_profiles_dropdown(self):
        profiles = list(self.model.get("saved_waypoints", {}).keys())
        if not profiles:
            profiles = ["No Saved Profiles"]
        self.saved_profiles_dropdown.clear()
        self.saved_profiles_dropdown.addItems(profiles)
        self.saved_profiles_dropdown.setCurrentText(profiles[-1])

    def save_current_waypoints(self):
        if not self.hover_poses:
            return
        text, ok = QtWidgets.QInputDialog.getText(self, "Save Waypoints", "Enter a name for this waypoint profile:")
        if ok and text:
            self.model["saved_waypoints"][text] = {"bolt_type": self.model["bolt_type"], "poses": self.hover_poses}
            self.save_model()
            self.update_profiles_dropdown()
            self.saved_profiles_dropdown.setCurrentText(text)
            self.log(f"[SYS] Profile '{text}' saved.")

    def load_selected_waypoints(self):
        profile_name = self.saved_profiles_dropdown.currentText()
        if profile_name in self.model.get("saved_waypoints", {}):
            data = self.model["saved_waypoints"][profile_name]
            if isinstance(data, list):
                self.hover_poses = list(data)
            else:
                self.hover_poses = list(data.get("poses", []))
                bolt_type = data.get("bolt_type", "M10")
                if bolt_type in BOLT_PROFILES:
                    self.model["bolt_type"] = bolt_type
                    self.model["torque_limit"] = BOLT_PROFILES[bolt_type]
                    self.bolt_dropdown.setCurrentText(bolt_type)
                    self.save_model()
            
            self.target_bolt_count = len(self.hover_poses)
            self.bolt_count_dropdown.setCurrentText(str(self.target_bolt_count))
            self.lbl_teach_status.setStyleSheet("color: #10B981; font-size: 14px; font-weight: bold;")
            self.lbl_teach_status.setText(f"Waypoints: {len(self.hover_poses)} / {self.target_bolt_count} - DEPLOY READY")
            self.log(f"[SYS] Loaded profile '{profile_name}'.")

    def _update_jog_speed(self, value):
        self.jog_speed = float(value)
        self.jog_speed_val_lbl.setText(f"{value} mm/s")

    # ── Position jog (hold = continuous motion, release = abort) ──────────
    def _start_pos_jog(self, axis, direction):
        if not self.is_connected:
            return
        axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]

        def task():
            res = self.robot.get_tcp_position()
            if res[0] != 0:
                return
            target = list(res[1])
            target[axis_index] += 600.0 * direction  # far target; abort stops it
            self.robot.linear_move(target, 0, False, self.jog_speed)

        threading.Thread(target=task, daemon=True).start()

    def _stop_pos_jog(self):
        try:
            self.robot.program_abort()
        except Exception:
            pass

    # ── Fixed orientation snap ─────────────────────────────────────────────
    def _set_fixed_orientation(self):
        if not self.is_connected:
            return
        def task():
            res = self.robot.get_tcp_position()
            if res[0] != 0:
                return
            pose = list(res[1])
            pose[3] = math.radians(180.0)   # Rx
            pose[4] = math.radians(0.0)     # Ry
            pose[5] = math.radians(180.0)   # Rz
            self.robot.linear_move(pose, 0, True, 30.0)
        threading.Thread(target=task, daemon=True).start()

    def emergency_stop(self):
        self.abort_flag = True
        self.log("\n[CRITICAL] EMERGENCY OVERRIDE INITIATED. HALTING FLEET.")
        try:
            self.robot.program_abort()
        except Exception:
            pass
        self.btn_run.setEnabled(True)

    # --- AUTONOMOUS SEQUENCE WITH CLOSED-LOOP WIGGLE ---
    def run_sequence(self):
        if not self.is_connected:
            self.log("[ERR] Robot offline.")
            return
        if len(self.hover_poses) != self.target_bolt_count:
            self.log(f"[ERR] Map all {self.target_bolt_count} waypoints.")
            return

        self.btn_run.setEnabled(False)
        self.abort_flag = False
        threading.Thread(target=self._execute_robot_task).start()

    def _smart_plunge(self, target_z_offset=-26.0):
        res_tcp = self.robot.get_tcp_position()
        if res_tcp[0] != 0:
            return False

        start_z = res_tcp[1][2]
        target_z = start_z + target_z_offset   # negative offset → moving down

        self.log(f"[OP] Smart plunge: {abs(target_z_offset):.0f} mm descent  "
                 f"Z {start_z:.1f} → {target_z:.1f}")

        plunge_pose = list(res_tcp[1])
        plunge_pose[2] = target_z

        DETECTION_ARM_DEPTH = 5.0
        STALL_INTERVAL = 0.5     # seconds between stall checks (long enough for SDK to update)
        STALL_MIN_MOVE = 1.5     # mm — arm moves 2.5 mm at 5 mm/s in 0.5 s; threshold is 60%
        STALL_CONFIRM = 2        # consecutive stall readings before triggering (= 1 s total)
        NEAR_TARGET = 5.0        # mm — stall within this of target = success (handles decel)
        detection_armed = False
        prev_z_check = start_z
        prev_z_time = 0.0
        stall_count = 0
        self.touch_detected = False

        self.robot.linear_move(plunge_pose, 0, False, 5.0)

        start_time = time.time()

        while not self.abort_flag:
            res = self.robot.get_tcp_position()
            if res[0] == 0:
                current_z = res[1][2]
                depth_descended = start_z - current_z

                # Arm sensor after first 5 mm so initial acceleration doesn't trigger it
                if not detection_armed and depth_descended >= DETECTION_ARM_DEPTH:
                    detection_armed = True
                    prev_z_check = current_z
                    prev_z_time = time.time()
                    self.touch_detected = False
                    self.log(f"[SYS] Contact sensor armed at {depth_descended:.1f} mm depth.")

                # ── SUCCESS PATH ─────────────────────────────────────────
                if abs(current_z - target_z) <= 1.0:
                    self.log("[SYS] Target depth reached.")
                    self.robot.program_abort()
                    return True

                # ── CONTACT via stall detection ───────────────────────────
                # Every STALL_INTERVAL s check how far Z moved.
                # STALL_MIN_MOVE set well below expected travel so only a real
                # stop (arm blocked) triggers it. STALL_CONFIRM consecutive
                # stall readings required to reject single-sample SDK noise.
                if detection_armed:
                    now = time.time()
                    if now - prev_z_time >= STALL_INTERVAL:
                        actual_move = abs(current_z - prev_z_check)
                        prev_z_check = current_z
                        prev_z_time = now
                        if actual_move < STALL_MIN_MOVE:
                            stall_count += 1
                            if stall_count >= STALL_CONFIRM:
                                self.touch_detected = True
                        else:
                            stall_count = 0

                if detection_armed and self.touch_detected:
                    self.touch_detected = False
                    detection_armed = False
                    stall_count = 0

                    if abs(current_z - target_z) <= NEAR_TARGET:
                        self.log("[SYS] Bolt contact at target depth — seated correctly.")
                        self.robot.program_abort()
                        return True

                    self.robot.program_abort()
                    time.sleep(0.5)

                    self.log(f"\n[WARN] Premature contact at {depth_descended:.1f} mm "
                             f"— misaligned. Wiggle recovery...")

                    # Z-only retract — keeps current J6 angle, does NOT reset it
                    self.log("       -> Retracting straight up (wrist locked)...")
                    res_tcp_r = self.robot.get_tcp_position()
                    if res_tcp_r[0] == 0:
                        retract = list(res_tcp_r[1])
                        retract[2] = start_z   # back to hover Z
                        self.robot.linear_move(retract, 0, True, 5.0)
                    time.sleep(0.2)

                    # Rotate +30° from wherever J6 currently is (cumulative across retries)
                    res_j = self.robot.get_joint_position()
                    if res_j[0] == 0:
                        wiggle_j = list(res_j[1])
                        wiggle_j[5] += math.radians(30)
                        self.log("       -> Rotating J6 +30° from current angle")
                        self.robot.joint_move(wiggle_j, 0, True, 5.0)

                    self.log("       -> Re-plunging to target depth...")
                    res_tcp2 = self.robot.get_tcp_position()
                    if res_tcp2[0] == 0:
                        resume_pose = list(res_tcp2[1])
                        resume_pose[2] = target_z
                        self.robot.linear_move(resume_pose, 0, False, 5.0)
                        prev_z_check = res_tcp2[1][2]

                    prev_z_time = time.time()
                    self.touch_detected = False
                    start_time = time.time()

            if time.time() - start_time > 30.0:
                self.log("[ERR] Plunge sequence timed out after 30 s.")
                self.robot.program_abort()
                return False

            time.sleep(0.01)

        return False

    def _execute_robot_task(self):
        try:
            self.log("\n--- INITIATING CONTINUOUS AUTONOMOUS SEQUENCE ---")
            self.log("[SYS] Running until Emergency Override is triggered.")
            cycle = 0

            while not self.abort_flag:
                cycle += 1
                self.log(f"\n========== CYCLE {cycle} ==========")

                for idx, pose in enumerate(self.hover_poses):
                    if self.abort_flag: break

                    self.log(f"\n[OP] Navigating to Target {idx + 1} Hover Pose...")
                    self.robot.joint_move(pose["joints"], 0, True, 1.0)
                    time.sleep(0.5)
                    if self.abort_flag: break

                    # Plunge 26 mm with closed-loop alignment
                    success = self._smart_plunge(-26.0)

                    if success and not self.abort_flag:
                        chk = self.robot.get_tcp_position()
                        if chk[0] != 0:
                            self.log("[WARN] Could not read TCP — skipping tighten.")
                        else:
                            depth_below = pose["cartesian"][2] - chk[1][2]
                            if depth_below < 20.0:
                                self.log(f"[WARN] Only {depth_below:.1f} mm deep — skipping tighten.")
                            else:
                                # Drive arm to exact tighten depth before rotating J6.
                                # Replaces the old sleep — this physically locks the arm
                                # at depth so it cannot drift between abort and tighten.
                                tighten_pose = list(chk[1])
                                tighten_pose[2] = pose["cartesian"][2] - 26.0
                                self.log(f"[SYS] {depth_below:.1f} mm deep. Locking tighten position...")
                                self.robot.linear_move(tighten_pose, 0, True, 2.0)

                                if not self.abort_flag:
                                    self.log("[SYS] Engaging 180 deg tightening rotation...")
                                    res_j = self.robot.get_joint_position()
                                    if res_j[0] == 0:
                                        spin_target = list(res_j[1])
                                        spin_target[5] += math.radians(180)
                                        self.robot.joint_move(spin_target, 0, True, 3.14)
                    else:
                        self.log("[ERR] Plunge sequence failed or aborted.")

                    if self.abort_flag: break

                    # Step 1: Retract straight up keeping current J6 angle.
                    # Do NOT move to pose["cartesian"] directly - it was recorded
                    # with J6=0 and linear_move interpolates orientation, rotating
                    # the bit inside the nut and unscrewing it during the lift.
                    self.log("[OP] Retracting straight up (wrist locked)...")
                    res_tcp = self.robot.get_tcp_position()
                    if res_tcp[0] == 0:
                        retract_pose = list(res_tcp[1])
                        retract_pose[2] = pose["cartesian"][2]  # hover Z, keep current Rx/Ry/Rz
                        self.robot.linear_move(retract_pose, 0, True, 5.0)
                    if self.abort_flag: break

                    # Step 2: Bit is clear of the nut - safe to reset J6 to 0.
                    self.log("[OP] Clear of bolt. Resetting wrist to 0 deg...")
                    res_j = self.robot.get_joint_position()
                    if res_j[0] == 0:
                        reset_j = list(res_j[1])
                        reset_j[5] = 0.0
                        self.robot.joint_move(reset_j, 0, True, 2.0)

                if not self.abort_flag:
                    self.log(f"\n[SYS] Cycle {cycle} complete. Restarting sequence...")

            self.log("\n--- OPERATIONS ABORTED VIA OVERRIDE ---")
            
        except Exception as e:
            self.log(f"[FATAL ERROR] System exception: {e}")
            self.emergency_stop()
            
        finally:
            self.btn_run.setEnabled(True)

    def closeEvent(self, event):
        self.is_connected = False
        self.telemetry_running = False
        self.abort_flag = True
        try:
            if hasattr(self, 'robot'): self.robot.logout()
        except: pass
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = BoltTighteningApp()
    window.show()
    sys.exit(app.exec_())