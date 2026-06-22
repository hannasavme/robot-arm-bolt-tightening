import time
import sys
import threading
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
from collections import deque
import jkrc

ROBOT_IP = "10.5.5.100"
TORQUE_JUMP_THRESHOLD = 4.0  

MAX_LIVE_POINTS = 300 
time_steps = deque(maxlen=MAX_LIVE_POINTS)
torque_history = [deque(maxlen=MAX_LIVE_POINTS) for _ in range(6)]

latest_torques = [0.0] * 6
touch_str = "None"
is_running = True
state = {'auto_scroll': True}

# --- THREAD 1: ULTRA-FAST DATA COLLECTION ---
def data_collection_worker():
    global touch_str, is_running, latest_torques
    
    robot = jkrc.RC(ROBOT_IP)
    robot.login("Administrator", "jakazuadmin")
    robot.power_on()
    robot.enable_robot()
    time.sleep(1.5)
    
    use_fast_mode = False
    try:
        if hasattr(robot, "get_joint_torque"):
            test_res = robot.get_joint_torque()
            if isinstance(test_res, (list, tuple)) and test_res[0] == 0:
                use_fast_mode = True
                print("[SYS] Fast-Torque SDK Mode ENABLED. Running at max speed.")
    except Exception:
        pass
        
    if not use_fast_mode:
        print("[SYS] Fast-Torque not supported. Falling back to Full Status Mode.")

    start_time = time.time()
    
    while is_running:
        try:
            if use_fast_mode:
                res = robot.get_joint_torque()
                if isinstance(res, (list, tuple)) and res[0] == 0:
                    latest_torques = [float(val) for val in res[1]]
            else:
                res = robot.get_robot_status()
                if isinstance(res, (list, tuple)) and res[0] == 0:
                    latest_torques = [float(res[1][20][5][i][4]) for i in range(6)]
            
            current_time = time.time() - start_time
            
            # Append data
            time_steps.append(current_time)
            for i in range(6):
                torque_history[i].append(latest_torques[i])

            affected_joints = []
            if len(time_steps) > 5:
                for i in range(6):
                    torque_diff = abs(latest_torques[i] - torque_history[i][-5])
                    if torque_diff >= TORQUE_JUMP_THRESHOLD:
                        affected_joints.append(f"J{i+1}")
            
            if affected_joints:
                touch_str = f"TOUCH on {', '.join(affected_joints)} (Time: {current_time:.1f}s)"

        except Exception as e:
            print(f"[DATA ERROR] {e}")
            
        time.sleep(0.01) 
        
    robot.logout()

# --- THREAD 2: HIGH-PERFORMANCE GUI RENDERING ---
def main():
    global is_running
    
    app = QtWidgets.QApplication(sys.argv)
    
    win = QtWidgets.QWidget()
    win.setWindowTitle("Areg.AI | High-Speed Interactive Torque Dashboard")
    win.resize(1000, 700)
    win.setStyleSheet("background-color: #0B0E14;") 
    
    layout = QtWidgets.QVBoxLayout()
    win.setLayout(layout)
    
    # --- UI PANEL: LIVE TEXT DASHBOARD ---
    dashboard_label = QtWidgets.QLabel("Waiting for telemetry...")
    # Use a monospace font so the numbers align perfectly like they did in the terminal
    dashboard_label.setStyleSheet("""
        color: #E2E8F0; 
        font-family: 'Courier New', Consolas, monospace; 
        font-size: 16px; 
        background-color: #151A22;
        padding: 15px;
        border-radius: 5px;
    """)
    layout.addWidget(dashboard_label)

    # --- UI PANEL: ALERTS ---
    alert_label = QtWidgets.QLabel("DETECTION EVENT: None")
    alert_label.setStyleSheet("color: #E2E8F0; font-size: 16px; font-weight: bold; margin-bottom: 5px; margin-top: 5px;")
    layout.addWidget(alert_label)
    
    # --- UI PANEL: THE GRAPH ---
    plot_widget = pg.PlotWidget()
    plot_widget.setBackground('#151A22')
    plot_widget.showGrid(x=True, y=True, alpha=0.3)
    plot_widget.setLabel('bottom', 'Time (Seconds)', color='#E2E8F0')
    plot_widget.setLabel('left', 'Torque (Nm)', color='#E2E8F0')
    plot_widget.addLegend()
    
    layout.addWidget(plot_widget)

    # --- UI PANEL: CONTROLS ---
    btn = QtWidgets.QPushButton('Pause Auto-Scroll')
    btn.setStyleSheet("""
        QPushButton {
            background-color: #F58220; 
            color: #151A22; 
            font-weight: bold; 
            padding: 10px; 
            border-radius: 5px;
        }
    """)
    layout.addWidget(btn)

    def toggle_scroll():
        state['auto_scroll'] = not state['auto_scroll']
        if state['auto_scroll']:
            btn.setText('Pause Auto-Scroll')
            btn.setStyleSheet("QPushButton {background-color: #F58220; color: #151A22; font-weight: bold; padding: 10px; border-radius: 5px;}")
        else:
            btn.setText('Resume Auto-Scroll')
            btn.setStyleSheet("QPushButton {background-color: #10B981; color: #151A22; font-weight: bold; padding: 10px; border-radius: 5px;}")
            
    btn.clicked.connect(toggle_scroll)

    joint_names = ["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)", "J4 (Wrist 1)", "J5 (Wrist 2)", "J6 (Wrist 3)"]
    colors = ['#FF5555', '#50FA7B', '#F1FA8C', '#8BE9FD', '#FF79C6', '#8A2BE2']
    
    curves = []
    for i in range(6):
        curve = plot_widget.plot(pen=pg.mkPen(colors[i], width=2), name=joint_names[i])
        curves.append(curve)

    data_thread = threading.Thread(target=data_collection_worker, daemon=True)
    data_thread.start()

    # --- 60 FPS UPDATE LOOP ---
    def update_ui():
        if not is_running:
            return
            
        # 1. Update the Text Dashboard
        dash_text = (
            f"  [Base]     J1: {latest_torques[0]:7.2f} Nm    |    [Wrist 1] J4: {latest_torques[3]:7.2f} Nm\n"
            f"  [Shoulder] J2: {latest_torques[1]:7.2f} Nm    |    [Wrist 2] J5: {latest_torques[4]:7.2f} Nm\n"
            f"  [Elbow]    J3: {latest_torques[2]:7.2f} Nm    |    [Wrist 3] J6: {latest_torques[5]:7.2f} Nm"
        )
        dashboard_label.setText(dash_text)

        # 2. Update the Graph
        snap_x = list(time_steps)
        snap_y = [list(th) for th in torque_history]
        
        min_len = min([len(snap_x)] + [len(y) for y in snap_y])
        if min_len < 2:
            return
            
        safe_x = snap_x[:min_len]
        
        for i in range(6):
            curves[i].setData(safe_x, snap_y[i][:min_len])
            
        # 3. Update the Alerts
        if "TOUCH" in touch_str:
            alert_label.setStyleSheet("color: #FF5555; font-size: 16px; font-weight: bold; margin-bottom: 5px; margin-top: 5px;")
        else:
            alert_label.setStyleSheet("color: #E2E8F0; font-size: 16px; font-weight: bold; margin-bottom: 5px; margin-top: 5px;")
        alert_label.setText(f"DETECTION EVENT: {touch_str}")
            
        # 4. Handle Camera Auto-Scroll
        if state['auto_scroll']:
            current_time = safe_x[-1]
            plot_widget.setXRange(max(0, current_time - 3.0), current_time + 0.2, padding=0)
            
            recent_vals = [val for y in snap_y for val in y]
            if recent_vals:
                plot_widget.setYRange(min(recent_vals) - 5, max(recent_vals) + 5, padding=0)

    timer = QtCore.QTimer()
    timer.timeout.connect(update_ui)
    timer.start(16)

    win.show()
    app.exec_()
    
    is_running = False
    print("\n[SYS] Sequence terminated cleanly.")

if __name__ == "__main__":
    main()