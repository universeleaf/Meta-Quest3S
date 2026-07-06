
import sys
import os
import serial
import time
import cv2
import csv
import numpy as np
import scipy.io
import onnxruntime as ort
from datetime import datetime
import pyqtgraph as pg

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QSlider, QLabel, QColorDialog, QFrame, QGridLayout, QGroupBox)
from PyQt5.QtGui import QColor, QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot

from quest_controller_pose_panel import QuestControllerPosePanel

# ==========================================
# 1. HARDWARE CONFIGURATION
# ==========================================
# LED Arduino Config
LED_SERIAL_PORT = '/dev/cu.usbmodem11101'  
LED_BAUD_RATE = 115200

# FT Sensor Config (UPDATE THESE PATHS)
FT_SERIAL_PORT = '/dev/cu.usbmodem1402' 
FT_BAUD_RATE = 1000000
FT_DATA_DIR = r'/Users/superteo/Desktop/pvft'  
FT_MODEL_FILE = 'PFT5-1_MLP_5L_norm_L2.onnx'
FT_NORM_FILE = 'PFT5-1_norm_constants.mat'


# ==========================================
# 2. THREAD: FORCE/TORQUE SENSOR
# ==========================================
class FTSensorThread(QThread):
    update_wrench = pyqtSignal(float, float, float, float, float, float)
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.START_BYTE = 2
        self.END_BYTE = 3
        
        self.read_count = 0
        self.initialSampleNum = 1500
        self.offset_CoinFT_list = []
        self.got_initial_offset = False
        self.offset_CoinFT = None
        
        self.ft_bias_ready = False
        self.ft_bias = np.zeros(6)
        self.request_tare = False

        self.single_capture_flag = False
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None

        try:
            onnx_path = os.path.join(FT_DATA_DIR, FT_MODEL_FILE)
            norm_path = os.path.join(FT_DATA_DIR, FT_NORM_FILE)
            self.ort_session = ort.InferenceSession(onnx_path)
            norm_data = scipy.io.loadmat(norm_path)
            self.mu_x = norm_data['norm_const']['mu_x'][0,0].flatten()
            self.sd_x = norm_data['norm_const']['sd_x'][0,0].flatten()
            self.mu_y = norm_data['norm_const']['mu_y'][0,0].flatten()
            self.sd_y = norm_data['norm_const']['sd_y'][0,0].flatten()
            self.model_loaded = True
        except Exception as e:
            self.model_loaded = False
            print(f"Failed to load FT Sensor Model: {e}")

    def run(self):
        if not self.model_loaded:
            self.status_message.emit("Model load failed. Check paths.")
            return

        try:
            self.ser = serial.Serial(FT_SERIAL_PORT, FT_BAUD_RATE, timeout=0.1)
            self.ser.write(b'i')
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.write(b'q')
            time.sleep(0.01)
            packet_size_excludeStartByte_raw = self.ser.read(1)
            if len(packet_size_excludeStartByte_raw) < 1:
                self.status_message.emit("Failed to read packet size from FT sensor.")
                return
            self.packet_size_excludeStartByte = ord(packet_size_excludeStartByte_raw) - 1
            self.ser.write(b's')
            self.status_message.emit("FT Sensor stream started. Collecting offset...")
        except Exception as e:
            self.status_message.emit(f"FT Serial Error: {e}")
            return

        while self._run_flag:
            byte = self.ser.read(1)
            if len(byte) == 0 or byte[0] != self.START_BYTE: continue
            data = self.ser.read(self.packet_size_excludeStartByte)
            if len(data) < self.packet_size_excludeStartByte: continue
            
            if data[-1] == self.END_BYTE:
                sensor_data = []
                for byte_num in range(0, self.packet_size_excludeStartByte-1, 2):
                    val = data[byte_num] + 256*data[byte_num+1]
                    sensor_data.append(val)
                sensor_data = np.array(sensor_data, dtype=np.float64)
                self.read_count += 1
                
                if self.read_count <= self.initialSampleNum:
                    self.offset_CoinFT_list.append(sensor_data)
                    if self.read_count % 300 == 0:
                        self.status_message.emit(f"Calibrating: {self.read_count}/{self.initialSampleNum}")
                    continue
                elif self.read_count == self.initialSampleNum + 1:
                    self.offset_CoinFT = np.mean(self.offset_CoinFT_list[5:], axis=0)
                    self.got_initial_offset = True
                    self.status_message.emit("Calibration complete. Streaming data.")
                    continue
                
                if not self.got_initial_offset: continue
                
                SensorData_offsetted = sensor_data - self.offset_CoinFT
                x_norm = (SensorData_offsetted - self.mu_x) / self.sd_x
                x_input = x_norm.astype(np.float32).reshape(1, -1)
                calibratedFT = self.ort_session.run(None, {"input": x_input})[0].flatten()
                calibratedFT = calibratedFT * self.sd_y + self.mu_y
                
                if not self.ft_bias_ready:
                    self.ft_bias = calibratedFT.copy()
                    self.ft_bias_ready = True
                if self.request_tare:
                    self.ft_bias = calibratedFT.copy()
                    self.request_tare = False
                    self.status_message.emit("Sensor Tared to Zero.")
                
                final_FT = calibratedFT - self.ft_bias
                
                if self.read_count % 10 == 0:
                    self.update_wrench.emit(float(final_FT[0]), float(final_FT[1]), float(final_FT[2]),
                                            float(final_FT[3]), float(final_FT[4]), float(final_FT[5]))

                if self.single_capture_flag:
                    os.makedirs("captures", exist_ok=True)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                    with open(f"captures/ft_data_{timestamp}.txt", 'w') as f:
                        f.write(f"Fx: {final_FT[0]:.4f}\nFy: {final_FT[1]:.4f}\nFz: {final_FT[2]:.4f}\n")
                        f.write(f"Tx: {final_FT[3]:.4f}\nTy: {final_FT[4]:.4f}\nTz: {final_FT[5]:.4f}\n")
                    self.single_capture_flag = False

                if self.is_recording and self.csv_writer:
                    self.csv_writer.writerow([time.time()] + final_FT.tolist())

    def tare_sensor(self): self.request_tare = True
    def capture_single_frame(self): self.single_capture_flag = True
    def toggle_recording(self, state):
        self.is_recording = state
        if state:
            os.makedirs("recordings", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_file = open(f"recordings/ft_data_{timestamp}.csv", 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
        else:
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None; self.csv_writer = None
    def stop(self):
        self._run_flag = False
        if self.csv_file: self.csv_file.close()
        if hasattr(self, 'ser') and self.ser.is_open: self.ser.close()
        self.wait()


# ==========================================
# 3. THREAD: VIDEO PROCESSING (ALL 5 FEATURES)
# ==========================================
class VideoThread(QThread):
    # ROW 1
    update_raw_frame = pyqtSignal(np.ndarray, float)
    update_crop_frame = pyqtSignal(np.ndarray)
    # ROW 2
    update_bgs_mask_raw_frame = pyqtSignal(np.ndarray)  
    update_bgs_mask_crop_frame = pyqtSignal(np.ndarray) 
    # ROW 3
    update_bgs_contour_raw_frame = pyqtSignal(np.ndarray)  
    update_bgs_contour_crop_frame = pyqtSignal(np.ndarray) 
    # ROW 4
    update_gel_contour_raw_frame = pyqtSignal(np.ndarray)  
    update_gel_contour_crop_frame = pyqtSignal(np.ndarray) 
    # ROW 5
    update_diff_frame = pyqtSignal(np.ndarray)
    update_crop_diff_frame = pyqtSignal(np.ndarray)
    
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.take_ref_flag = False
        self.reference_frame = None
        self.reference_crop_frame = None
        
        self.back_sub_raw = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=True)
        self.back_sub_crop = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=True)

        self.single_capture_flag = False
        self.is_recording = False
        self.video_writers = None

    def compute_tactile_diff(self, img_current, img_bg, offset=0.5):
        img1 = np.int32(img_current)
        img2 = np.int32(img_bg)
        diff = img1 - img2
        diff = diff / 255.0 + offset
        diff = np.clip(diff, 0.0, 1.0)
        return np.uint8(diff * 255.0)

    def run(self):
        cap = cv2.VideoCapture(0)
        prev_time = time.time()

        while self._run_flag:
            ret, frame = cap.read()
            if not ret: continue

            current_time = time.time()
            fps = 1.0 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
            prev_time = current_time

            # --- ROW 1: Raw & Crop ---
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.update_raw_frame.emit(rgb_frame, fps)

            h, w, _ = rgb_frame.shape
            crop_size = min(h, w)
            start_x = (w - crop_size) // 2
            start_y = (h - crop_size) // 2
            cropped = rgb_frame[start_y:start_y+crop_size, start_x:start_x+crop_size]
            resized_224 = cv2.resize(cropped, (224, 224), interpolation=cv2.INTER_AREA)
            self.update_crop_frame.emit(resized_224)

            # ---------------------------------------------------------
            # MOG2 Mask Calculation
            # ---------------------------------------------------------
            bgs_raw_mask = self.back_sub_raw.apply(rgb_frame)
            bgs_crop_mask = self.back_sub_crop.apply(resized_224)

            # --- ROW 2: Pure MOG2 Masks ---
            bgs_mask_raw_rgb = cv2.cvtColor(bgs_raw_mask, cv2.COLOR_GRAY2RGB)
            bgs_mask_crop_rgb = cv2.cvtColor(bgs_crop_mask, cv2.COLOR_GRAY2RGB)
            self.update_bgs_mask_raw_frame.emit(bgs_mask_raw_rgb)
            self.update_bgs_mask_crop_frame.emit(bgs_mask_crop_rgb)

            # ---------------------------------------------------------
            # Contour Extractions
            # ---------------------------------------------------------
            _, thresh_raw = cv2.threshold(bgs_raw_mask, 200, 255, cv2.THRESH_BINARY)
            contours_raw, _ = cv2.findContours(thresh_raw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            _, thresh_crop = cv2.threshold(bgs_crop_mask, 200, 255, cv2.THRESH_BINARY)
            contours_crop, _ = cv2.findContours(thresh_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # --- ROW 3: BGS + Green Contours (Color Background) ---
            bgs_contour_raw = rgb_frame.copy()
            cv2.drawContours(bgs_contour_raw, contours_raw, -1, (0, 255, 0), 2)
            
            bgs_contour_crop = resized_224.copy()
            cv2.drawContours(bgs_contour_crop, contours_crop, -1, (0, 255, 0), 2)
            
            self.update_bgs_contour_raw_frame.emit(bgs_contour_raw)
            self.update_bgs_contour_crop_frame.emit(bgs_contour_crop)

            # --- ROW 4: Gel Contour (Black & White) ---
            gel_contour_raw = np.zeros_like(rgb_frame)
            cv2.drawContours(gel_contour_raw, contours_raw, -1, (255, 255, 255), 2)
            
            gel_contour_crop = np.zeros_like(resized_224)
            cv2.drawContours(gel_contour_crop, contours_crop, -1, (255, 255, 255), 2)

            self.update_gel_contour_raw_frame.emit(gel_contour_raw)
            self.update_gel_contour_crop_frame.emit(gel_contour_crop)

            # --- ROW 5: Tactile Diff (Gray) ---
            if self.take_ref_flag:
                self.reference_frame = rgb_frame.copy()
                self.reference_crop_frame = resized_224.copy()
                self.take_ref_flag = False
                self.status_message.emit("Gray Reference frame captured.")

            diff_frame = None
            diff_crop_frame = None

            if self.reference_frame is not None:
                diff_frame = self.compute_tactile_diff(rgb_frame, self.reference_frame)
                self.update_diff_frame.emit(diff_frame)
                
            if self.reference_crop_frame is not None:
                diff_crop_frame = self.compute_tactile_diff(resized_224, self.reference_crop_frame)
                self.update_crop_diff_frame.emit(diff_crop_frame)

            # ---------------------------------------------------------
            # DATA COLLECTION
            # ---------------------------------------------------------
            if self.single_capture_flag:
                # Save all 10 visual states
                self._save_single_capture(
                    frame, resized_224, 
                    bgs_mask_raw_rgb, bgs_mask_crop_rgb,
                    bgs_contour_raw, bgs_contour_crop,
                    gel_contour_raw, gel_contour_crop,
                    diff_frame, diff_crop_frame
                )
                self.single_capture_flag = False

            if self.is_recording:
                if self.video_writers is None:
                    os.makedirs("recordings", exist_ok=True)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    self.video_writers = {
                        'raw': cv2.VideoWriter(f"recordings/raw_{timestamp}.avi", fourcc, 20.0, (w, h)),
                        'crop': cv2.VideoWriter(f"recordings/crop_{timestamp}.avi", fourcc, 20.0, (224, 224))
                    }
                    self.status_message.emit(f"Recording started: {timestamp}")
                
                self.video_writers['raw'].write(frame)
                self.video_writers['crop'].write(cv2.cvtColor(resized_224, cv2.COLOR_RGB2BGR))
            else:
                if self.video_writers is not None:
                    self.video_writers['raw'].release(); self.video_writers['crop'].release(); self.video_writers = None
                    self.status_message.emit("Recording stopped and saved.")

        if self.video_writers is not None:
            self.video_writers['raw'].release(); self.video_writers['crop'].release()
        cap.release()

    def _save_single_capture(self, raw, crop, mask_raw, mask_crop, cont_raw, cont_crop, gel_raw, gel_crop, diff_raw, diff_crop):
        os.makedirs("captures", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3] 
        
        cv2.imwrite(f"captures/1_raw_{timestamp}.jpg", raw)
        cv2.imwrite(f"captures/1_crop_{timestamp}.jpg", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/2_mask_raw_{timestamp}.jpg", cv2.cvtColor(mask_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/2_mask_crop_{timestamp}.jpg", cv2.cvtColor(mask_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/3_bgs_contour_raw_{timestamp}.jpg", cv2.cvtColor(cont_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/3_bgs_contour_crop_{timestamp}.jpg", cv2.cvtColor(cont_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/4_gel_contour_raw_{timestamp}.jpg", cv2.cvtColor(gel_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/4_gel_contour_crop_{timestamp}.jpg", cv2.cvtColor(gel_crop, cv2.COLOR_RGB2BGR))
        
        if diff_raw is not None: cv2.imwrite(f"captures/5_diff_gray_raw_{timestamp}.jpg", cv2.cvtColor(diff_raw, cv2.COLOR_RGB2BGR))
        if diff_crop is not None: cv2.imwrite(f"captures/5_diff_gray_crop_{timestamp}.jpg", cv2.cvtColor(diff_crop, cv2.COLOR_RGB2BGR))
            
        self.status_message.emit(f"Captured all 10 frames at {timestamp}")

    def capture_reference(self): self.take_ref_flag = True
    def capture_single_frame(self): self.single_capture_flag = True
    def toggle_recording(self, state): self.is_recording = state
    def stop(self): self._run_flag = False; self.wait()


# ==========================================
# 4. UI COMPONENTS (LEDs, FT Plots)
# ==========================================
# [LEDControlPanel and FTSensorPanel remain exactly the same functionally]
class LEDControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.num_leds = 6
        self.led_colors = [(0, 0, 0) for _ in range(self.num_leds)]
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1)
            time.sleep(2)
        except: pass
        self.init_ui()
        self.send_serial_command(99, 20, 0, 0)
        self.set_default_rgbrgb()
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("Part 1: LED Controller"); title.setFont(QFont("Arial", 14, QFont.Bold)); layout.addWidget(title)
        brightness_layout = QVBoxLayout()
        self.brightness_label = QLabel("Global Brightness: 20")
        self.brightness_slider = QSlider(Qt.Horizontal); self.brightness_slider.setRange(0, 255); self.brightness_slider.setValue(20)
        self.brightness_slider.valueChanged.connect(self.on_brightness_changed)
        brightness_layout.addWidget(self.brightness_label); brightness_layout.addWidget(self.brightness_slider); layout.addLayout(brightness_layout)
        led_layout = QHBoxLayout()
        self.led_buttons = []
        for i in range(self.num_leds):
            btn = QPushButton(f"LED {i}"); btn.setFixedSize(60, 60); btn.clicked.connect(lambda checked, idx=i: self.choose_color(idx)); self.led_buttons.append(btn); led_layout.addWidget(btn)
        layout.addLayout(led_layout)
        quick_layout = QHBoxLayout()
        btn_red = QPushButton("All Red"); btn_red.clicked.connect(lambda: self.set_all_color(255, 0, 0))
        btn_green = QPushButton("All Green"); btn_green.clicked.connect(lambda: self.set_all_color(0, 255, 0))
        btn_blue = QPushButton("All Blue"); btn_blue.clicked.connect(lambda: self.set_all_color(0, 0, 255))
        btn_def = QPushButton("Default"); btn_def.clicked.connect(self.set_default_rgbrgb)
        btn_off = QPushButton("Turn Off All"); btn_off.clicked.connect(lambda: self.set_all_color(0, 0, 0))
        for b in (btn_red, btn_green, btn_blue, btn_def, btn_off): quick_layout.addWidget(b)
        layout.addLayout(quick_layout); layout.addStretch(); self.setLayout(layout)
    def send_serial_command(self, index, r, g, b):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.write(f"{index},{r},{g},{b}\n".encode('utf-8')); time.sleep(0.01)
    def update_button_color(self, idx, r, g, b):
        style = f"background-color: #{r:02x}{g:02x}{b:02x}; color: {'black' if (0.299*r+0.587*g+0.114*b)>128 else 'white'}; font-weight: bold; border-radius: 5px;"
        self.led_buttons[idx].setStyleSheet(style)
    def on_brightness_changed(self):
        val = self.brightness_slider.value(); self.brightness_label.setText(f"Global Brightness: {val}"); self.send_serial_command(99, val, 0, 0)
    def choose_color(self, idx):
        c = QColorDialog.getColor(QColor(*self.led_colors[idx]), self, f"LED {idx}")
        if c.isValid(): self.set_single_led(idx, c.red(), c.green(), c.blue())
    def set_single_led(self, idx, r, g, b): self.led_colors[idx] = (r, g, b); self.update_button_color(idx, r, g, b); self.send_serial_command(idx, r, g, b)
    def set_all_color(self, r, g, b):
        for i in range(self.num_leds): self.set_single_led(i, r, g, b)
    def set_default_rgbrgb(self):
        p = [(255,0,0), (255,0,0),(0,255,0), (0,255,0), (0,0,255),  (0,0,255)]
        for i in range(self.num_leds): self.set_single_led(i, *p[i])
    def closeEvent(self, e):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()

class FTSensorPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.history_size = 150
        self.data_fx = np.zeros(self.history_size); self.data_fy = np.zeros(self.history_size); self.data_fz = np.zeros(self.history_size)
        self.data_tx = np.zeros(self.history_size); self.data_ty = np.zeros(self.history_size); self.data_tz = np.zeros(self.history_size)
        self.ft_thread = FTSensorThread(); self.init_ui(); self.ft_thread.update_wrench.connect(self.update_display); self.ft_thread.status_message.connect(self.update_status); self.ft_thread.start()
    def init_ui(self):
        layout = QVBoxLayout()
        header_layout = QHBoxLayout()
        title = QLabel("Part 4: CoinFT Sensor"); title.setFont(QFont("Arial", 14, QFont.Bold)); header_layout.addWidget(title)
        self.btn_tare = QPushButton("Tare (Zero) Sensor"); self.btn_tare.setStyleSheet("background-color: orange; font-weight: bold;"); self.btn_tare.clicked.connect(self.ft_thread.tare_sensor); header_layout.addWidget(self.btn_tare); layout.addLayout(header_layout)
        self.status_label = QLabel("Status: Waiting..."); self.status_label.setStyleSheet("color: blue; font-style: italic;"); layout.addWidget(self.status_label)
        readout_layout = QHBoxLayout()
        force_group = QGroupBox("Forces (N)"); force_layout = QHBoxLayout()
        self.lbl_fx = QLabel("Fx: 0.00"); self.lbl_fy = QLabel("Fy: 0.00"); self.lbl_fz = QLabel("Fz: 0.00")
        for lbl in (self.lbl_fx, self.lbl_fy, self.lbl_fz): lbl.setFont(QFont("Consolas", 12, QFont.Bold)); force_layout.addWidget(lbl)
        force_group.setLayout(force_layout)
        torque_group = QGroupBox("Torques (Nm)"); torque_layout = QHBoxLayout()
        self.lbl_tx = QLabel("Tx: 0.000"); self.lbl_ty = QLabel("Ty: 0.000"); self.lbl_tz = QLabel("Tz: 0.000")
        for lbl in (self.lbl_tx, self.lbl_ty, self.lbl_tz): lbl.setFont(QFont("Consolas", 12, QFont.Bold)); torque_layout.addWidget(lbl)
        torque_group.setLayout(torque_layout); readout_layout.addWidget(force_group); readout_layout.addWidget(torque_group); layout.addLayout(readout_layout)
        pg.setConfigOptions(antialias=True); self.plot_f = pg.PlotWidget(title="Force (N)"); self.plot_f.setFixedHeight(180); self.plot_f.addLegend(); self.plot_f.showGrid(x=False, y=True)
        self.curve_fx = self.plot_f.plot(pen=pg.mkPen('r', width=2), name="Fx"); self.curve_fy = self.plot_f.plot(pen=pg.mkPen('g', width=2), name="Fy"); self.curve_fz = self.plot_f.plot(pen=pg.mkPen('c', width=2), name="Fz")
        self.plot_t = pg.PlotWidget(title="Torque (Nm)"); self.plot_t.setFixedHeight(180); self.plot_t.addLegend(); self.plot_t.showGrid(x=False, y=True)
        self.curve_tx = self.plot_t.plot(pen=pg.mkPen('r', width=2), name="Tx"); self.curve_ty = self.plot_t.plot(pen=pg.mkPen('g', width=2), name="Ty"); self.curve_tz = self.plot_t.plot(pen=pg.mkPen('c', width=2), name="Tz")
        layout.addWidget(self.plot_f); layout.addWidget(self.plot_t); self.setLayout(layout)
    @pyqtSlot(str)
    def update_status(self, msg): self.status_label.setText(f"Status: {msg}")
    @pyqtSlot(float, float, float, float, float, float)
    def update_display(self, fx, fy, fz, tx, ty, tz):
        for lbl, val, fmt in ((self.lbl_fx, fx, "6.2f"), (self.lbl_fy, fy, "6.2f"), (self.lbl_fz, fz, "6.2f"), (self.lbl_tx, tx, "6.3f"), (self.lbl_ty, ty, "6.3f"), (self.lbl_tz, tz, "6.3f")): lbl.setText(f"{lbl.text().split(':')[0]}: {val:{fmt}}")
        for data, val in ((self.data_fx, fx), (self.data_fy, fy), (self.data_fz, fz), (self.data_tx, tx), (self.data_ty, ty), (self.data_tz, tz)): data[:-1] = data[1:]; data[-1] = val
        self.curve_fx.setData(self.data_fx); self.curve_fy.setData(self.data_fy); self.curve_fz.setData(self.data_fz); self.curve_tx.setData(self.data_tx); self.curve_ty.setData(self.data_ty); self.curve_tz.setData(self.data_tz)
    def closeEvent(self, event): self.ft_thread.stop()

# ==========================================
# 5. UI COMPONENTS: VISION & DATA COLLECT (5x2 GRID)
# ==========================================
class CameraMonitorPanel(QWidget):
    req_capture = pyqtSignal()
    req_record = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.video_thread = VideoThread()
        self.init_ui()
        
        # Row 1
        self.video_thread.update_raw_frame.connect(self.update_raw_image)
        self.video_thread.update_crop_frame.connect(self.update_crop_image)
        # Row 2
        self.video_thread.update_bgs_mask_raw_frame.connect(self.update_bgs_mask_raw_image)
        self.video_thread.update_bgs_mask_crop_frame.connect(self.update_bgs_mask_crop_image)
        # Row 3
        self.video_thread.update_bgs_contour_raw_frame.connect(self.update_bgs_contour_raw_image)
        self.video_thread.update_bgs_contour_crop_frame.connect(self.update_bgs_contour_crop_image)
        # Row 4
        self.video_thread.update_gel_contour_raw_frame.connect(self.update_gel_contour_raw_image)
        self.video_thread.update_gel_contour_crop_frame.connect(self.update_gel_contour_crop_image)
        # Row 5
        self.video_thread.update_diff_frame.connect(self.update_diff_image)
        self.video_thread.update_crop_diff_frame.connect(self.update_crop_diff_image)
        
        self.video_thread.status_message.connect(self.update_status)
        self.video_thread.start()

    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("Part 2 & 3: Vision Analytics Matrix (5x2)"); title.setFont(QFont("Arial", 14, QFont.Bold)); layout.addWidget(title)

        grid = QGridLayout(); grid.setSpacing(5)

        # Removed FixedSize to allow window to shrink/scroll if needed, while maintaining ratios
        def create_video_label():
            lbl = QLabel()
            lbl.setMinimumSize(160, 120) 
            lbl.setStyleSheet("background-color: black; border: 1px solid gray;")
            lbl.setAlignment(Qt.AlignCenter)
            return lbl
            
        def create_title(text):
            lbl = QLabel(text); lbl.setFont(QFont("Arial", 9, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter); return lbl

        # ROW 1
        self.label_raw_title = create_title("1. RAW Input (FPS: --)"); self.label_raw_video = create_video_label()
        self.label_crop_title = create_title("ML Crop (224x224)"); self.label_crop_video = create_video_label()
        grid.addWidget(self.label_raw_title, 0, 0); grid.addWidget(self.label_raw_video, 1, 0)
        grid.addWidget(self.label_crop_title, 0, 1); grid.addWidget(self.label_crop_video, 1, 1)

        # ROW 2
        self.label_bgs_mask_raw_title = create_title("2. BGS Mask (MOG2 RAW)"); self.label_bgs_mask_raw_video = create_video_label()
        self.label_bgs_mask_crop_title = create_title("BGS Mask (MOG2 Crop)"); self.label_bgs_mask_crop_video = create_video_label()
        grid.addWidget(self.label_bgs_mask_raw_title, 2, 0); grid.addWidget(self.label_bgs_mask_raw_video, 3, 0)
        grid.addWidget(self.label_bgs_mask_crop_title, 2, 1); grid.addWidget(self.label_bgs_mask_crop_video, 3, 1)

        # ROW 3
        self.label_bgs_cont_raw_title = create_title("3. BGS + Green Contours"); self.label_bgs_cont_raw_video = create_video_label()
        self.label_bgs_cont_crop_title = create_title("BGS + Green Contours"); self.label_bgs_cont_crop_video = create_video_label()
        grid.addWidget(self.label_bgs_cont_raw_title, 4, 0); grid.addWidget(self.label_bgs_cont_raw_video, 5, 0)
        grid.addWidget(self.label_bgs_cont_crop_title, 4, 1); grid.addWidget(self.label_bgs_cont_crop_video, 5, 1)

        # ROW 4
        self.label_gel_cont_raw_title = create_title("4. Gel Contour (B&W RAW)"); self.label_gel_cont_raw_video = create_video_label()
        self.label_gel_cont_crop_title = create_title("Gel Contour (B&W Crop)"); self.label_gel_cont_crop_video = create_video_label()
        grid.addWidget(self.label_gel_cont_raw_title, 6, 0); grid.addWidget(self.label_gel_cont_raw_video, 7, 0)
        grid.addWidget(self.label_gel_cont_crop_title, 6, 1); grid.addWidget(self.label_gel_cont_crop_video, 7, 1)

        # ROW 5
        self.label_diff_title = create_title("5. Tactile Diff (Gray Ref)"); self.label_diff_video = create_video_label()
        self.label_crop_diff_title = create_title("Tactile Diff (Gray Ref)"); self.label_crop_diff_video = create_video_label()
        grid.addWidget(self.label_diff_title, 8, 0); grid.addWidget(self.label_diff_video, 9, 0)
        grid.addWidget(self.label_crop_diff_title, 8, 1); grid.addWidget(self.label_crop_diff_video, 9, 1)

        layout.addLayout(grid); layout.addSpacing(10)

        # Controls
        controls_layout = QHBoxLayout()
        self.btn_take_ref = QPushButton("Take Static Ref"); self.btn_take_ref.setMinimumHeight(40); self.btn_take_ref.clicked.connect(self.video_thread.capture_reference)
        self.btn_capture = QPushButton("Capture Snap (10 Frames + FT)"); self.btn_capture.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;"); self.btn_capture.setMinimumHeight(40); self.btn_capture.clicked.connect(self.on_capture)
        self.btn_record = QPushButton("Start Recording"); self.btn_record.setMinimumHeight(40); self.btn_record.setCheckable(True); self.btn_record.setStyleSheet("QPushButton:checked { background-color: red; color: white; font-weight: bold; }"); self.btn_record.toggled.connect(self.on_record)
        controls_layout.addWidget(self.btn_take_ref); controls_layout.addWidget(self.btn_capture); controls_layout.addWidget(self.btn_record)
        layout.addLayout(controls_layout)

        self.status_label = QLabel("Status: Ready"); self.status_label.setStyleSheet("color: blue; font-style: italic;")
        layout.addWidget(self.status_label); layout.addStretch(); self.setLayout(layout)

    def on_capture(self): self.video_thread.capture_single_frame(); self.req_capture.emit() 
    def on_record(self, checked):
        self.btn_record.setText("Stop Recording" if checked else "Start Recording")
        self.video_thread.toggle_recording(checked); self.req_record.emit(checked) 

    @pyqtSlot(str)
    def update_status(self, msg): self.status_label.setText(f"Status: {msg}")

    def convert_cv_qt(self, cv_img, target_width=320, target_height=240):
        qh, qw, ch = cv_img.shape; bytes_per_line = ch * qw
        q_img = QImage(cv_img.data, qw, qh, bytes_per_line, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(q_img)
        return pixmap.scaled(target_width, target_height, Qt.KeepAspectRatio)

    # --- SLOTS FOR ALL 10 STREAMS ---
    @pyqtSlot(np.ndarray, float)
    def update_raw_image(self, cv_img, fps):
        self.label_raw_title.setText(f"1. RAW Input (FPS: {fps:.1f})")
        self.label_raw_video.setPixmap(self.convert_cv_qt(cv_img))

    @pyqtSlot(np.ndarray)
    def update_crop_image(self, cv_img): self.label_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))
    
    @pyqtSlot(np.ndarray)
    def update_bgs_mask_raw_image(self, cv_img): self.label_bgs_mask_raw_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_bgs_mask_crop_image(self, cv_img): self.label_bgs_mask_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_bgs_contour_raw_image(self, cv_img): self.label_bgs_cont_raw_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_bgs_contour_crop_image(self, cv_img): self.label_bgs_cont_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_gel_contour_raw_image(self, cv_img): self.label_gel_cont_raw_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_gel_contour_crop_image(self, cv_img): self.label_gel_cont_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_diff_image(self, cv_img): self.label_diff_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_crop_diff_image(self, cv_img): self.label_crop_diff_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    def closeEvent(self, event): self.video_thread.stop()

# ==========================================
# 6. MAIN DASHBOARD (Integration)
# ==========================================
class MainApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vision & CoinFT Ultimate Advanced Analytics Dashboard (5x2 Grid)')
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        self.led_panel = LEDControlPanel(); self.quest_pose_panel = QuestControllerPosePanel(); self.ft_panel = FTSensorPanel(); self.camera_panel = CameraMonitorPanel()
        
        self.camera_panel.req_capture.connect(self.ft_panel.ft_thread.capture_single_frame)
        self.camera_panel.req_record.connect(self.ft_panel.ft_thread.toggle_recording)

        left_layout.addWidget(self.led_panel); left_layout.addWidget(self.quest_pose_panel); left_layout.addWidget(self.ft_panel)
        
        line = QFrame(); line.setFrameShape(QFrame.VLine); line.setFrameShadow(QFrame.Sunken)
        main_layout.addLayout(left_layout); main_layout.addWidget(line); main_layout.addWidget(self.camera_panel)
        self.setLayout(main_layout)

    def closeEvent(self, event):
        self.camera_panel.closeEvent(event); self.led_panel.closeEvent(event); self.quest_pose_panel.closeEvent(event); self.ft_panel.closeEvent(event)
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv); app.setStyle('Fusion')
    window = MainApp()
    
    # Optional: Maximize window on start since we have 10 video feeds
    window.showMaximized() 
    
    sys.exit(app.exec_())
