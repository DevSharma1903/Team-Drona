#!/usr/bin/env python3
"""
Ball Balancing Platform - Raspberry Pi Camera 
"""

import cv2
import numpy as np
import serial
import csv
import time
import sys
import os
import threading
from queue import Queue, Full
from datetime import datetime
from pathlib import Path
from collections import deque

# ============================================================================
# CONFIGURATION
# ============================================================================
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 420
CAMERA_FPS_TARGET = 120
CAMERA_INDEX = 0

PLATE_WIDTH_MM = 76.7
PLATE_HEIGHT_MM = 63.65

HSV_LOWER_1 = np.array([0, 120, 100])
HSV_UPPER_1 = np.array([10, 255, 255])
HSV_LOWER_2 = np.array([170, 120, 100])
HSV_UPPER_2 = np.array([180, 255, 255])

MIN_BALL_AREA_PX = 200
MAX_BALL_AREA_PX = 3000
MIN_CIRCULARITY = 0.6
MORPH_KERNEL_SIZE = 5

HOUGH_DP = 1.2
HOUGH_MIN_DIST = 50
HOUGH_PARAM1 = 50
HOUGH_PARAM2 = 30
HOUGH_MIN_RADIUS = 8
HOUGH_MAX_RADIUS = 35

UART_PORT = '/dev/ttyS0'
UART_BAUD = 115200
UART_TIMEOUT = 0.05
UART_STX = 0x02
UART_ETX = 0x03

# NEW: UART buffer safety limit
MAX_UART_BYTES_PER_READ = 512  # Prevent runaway loop
MAX_UART_PACKET_LENGTH = 200  # Teensy packets are ~100 bytes max (safety limit)

DETECTION_TIME_WARNING_MS = 50.0
HSV_FAILURE_THRESHOLD = 0.5

LOG_DIR = Path(os.environ.get('BALL_BALANCER_LOG_DIR', '/tmp/ball_balancer_logs'))
LOG_DIR.mkdir(parents=True, exist_ok=True)
MAX_LOG_ROWS = 100000

SAVE_FRAMES = True
FRAME_SAVE_DIR = LOG_DIR / 'frames'
FRAME_SAVE_DIR.mkdir(parents=True, exist_ok=True)
FRAME_SAVE_INTERVAL = 30

# ============================================================================
# SAFE FORMATTING HELPER
# ============================================================================
def safe_format_float(value, decimal_places=1):
    """Format float safely, handling NaN/Inf"""
    if value is None or np.isnan(value) or np.isinf(value):
        return "N/A"
    return f"{value:.{decimal_places}f}"

# ============================================================================
# CRC16
# ============================================================================
def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

# ============================================================================
# BALL DETECTOR
# ============================================================================
class BallDetector:
    """Detects red ball with HSV + Hough Circle fallback"""
    
    def __init__(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
        self.center_x = CAMERA_WIDTH // 2
        self.center_y = CAMERA_HEIGHT // 2
        
        self.scale_x = PLATE_WIDTH_MM / CAMERA_WIDTH
        self.scale_y = PLATE_HEIGHT_MM / CAMERA_HEIGHT
        
        self.last_x_px = None
        self.last_y_px = None
        self.alpha = 0.6
        
        self.detection_times = deque(maxlen=100)
        self.hsv_success_count = 0
        self.hough_success_count = 0
        self.hsv_attempts = 0
        self.performance_warnings = 0
        
        self.prefer_hough = False
        self.method_check_interval = 100
        self.frame_counter = 0
    
    def detect(self, frame):
        """Detect ball using HSV or Hough"""
        start_time = time.time()
        self.frame_counter += 1
        
        if self.prefer_hough:
            result = self._detect_hough(frame)
            if result[0] is None:
                result = self._detect_hsv(frame)
        else:
            result = self._detect_hsv(frame)
            if result[0] is None:
                result = self._detect_hough(frame)
        
        process_time = (time.time() - start_time) * 1000
        self.detection_times.append(process_time)
        
        if process_time > DETECTION_TIME_WARNING_MS:
            self.performance_warnings += 1
            if self.performance_warnings % 10 == 0:
                print(f"⚠ Detection slow: {process_time:.1f}ms")
        
        if self.frame_counter % self.method_check_interval == 0:
            self._update_method_preference()
        
        return result
    
    def _update_method_preference(self):
        """Switch to Hough if HSV is failing too often"""
        if self.hsv_attempts > 0:
            hsv_success_rate = self.hsv_success_count / self.hsv_attempts
            
            if hsv_success_rate < HSV_FAILURE_THRESHOLD and not self.prefer_hough:
                print(f"📊 Switching to Hough (HSV: {hsv_success_rate*100:.1f}%)")
                self.prefer_hough = True
            elif hsv_success_rate >= (HSV_FAILURE_THRESHOLD + 0.1) and self.prefer_hough:
                print(f"📊 Switching to HSV (HSV: {hsv_success_rate*100:.1f}%)")
                self.prefer_hough = False
    
    def _detect_hsv(self, frame):
        """Primary detection using HSV"""
        self.hsv_attempts += 1
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, HSV_LOWER_1, HSV_UPPER_1)
        mask2 = cv2.inRange(hsv, HSV_LOWER_2, HSV_UPPER_2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_score = 0
        best_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if MIN_BALL_AREA_PX < area < MAX_BALL_AREA_PX:
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    if circularity > MIN_CIRCULARITY:
                        score = area * circularity
                        
                        if score > best_score:
                            best_score = score
                            best_area = area
                            best_contour = contour
        
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                radius = int(np.sqrt(best_area / np.pi))
                
                ball_x_mm, ball_y_mm = self._apply_filtering_and_convert(cx, cy)
                self.hsv_success_count += 1
                
                return (ball_x_mm, ball_y_mm, cx, cy, radius, mask, 'HSV')
        
        return (None, None, None, None, None, mask, None)
    
    def _detect_hough(self, frame):
        """Fallback detection using Hough Circle"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=HOUGH_DP,
            minDist=HOUGH_MIN_DIST,
            param1=HOUGH_PARAM1,
            param2=HOUGH_PARAM2,
            minRadius=HOUGH_MIN_RADIUS,
            maxRadius=HOUGH_MAX_RADIUS
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            cx, cy, radius = circles[0]
            
            ball_x_mm, ball_y_mm = self._apply_filtering_and_convert(cx, cy)
            self.hough_success_count += 1
            
            mask = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH), dtype=np.uint8)
            return (ball_x_mm, ball_y_mm, cx, cy, radius, mask, 'Hough')
        
        return (None, None, None, None, None, 
                np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH), dtype=np.uint8), None)
    
    def _apply_filtering_and_convert(self, cx, cy):
        """Apply temporal filter and convert to mm"""
        if self.last_x_px is None:
            self.last_x_px = cx
            self.last_y_px = cy
        else:
            cx = self.alpha * cx + (1 - self.alpha) * self.last_x_px
            cy = self.alpha * cy + (1 - self.alpha) * self.last_y_px
            self.last_x_px = cx
            self.last_y_px = cy
        
        ball_x_mm = (cx - self.center_x) * self.scale_x + PLATE_WIDTH_MM / 2
        ball_y_mm = (cy - self.center_y) * self.scale_y + PLATE_HEIGHT_MM / 2
        
        return ball_x_mm, ball_y_mm
    
    def get_detection_stats(self):
        """Return detection statistics"""
        total = self.hsv_success_count + self.hough_success_count
        if total == 0:
            return "Detection: N/A"
        
        hsv_pct = (self.hsv_success_count / total) * 100
        hough_pct = (self.hough_success_count / total) * 100
        method = "Hough" if self.prefer_hough else "HSV"
        
        return f"Detection: {hsv_pct:.0f}% HSV, {hough_pct:.0f}% Hough [Prefer: {method}]"
    
    def get_avg_process_time(self):
        """Return average processing time"""
        if len(self.detection_times) == 0:
            return 0.0
        return sum(self.detection_times) / len(self.detection_times)
    
    def reset_stats(self):
        """Reset statistics"""
        self.hsv_success_count = 0
        self.hough_success_count = 0
        self.hsv_attempts = 0
        self.detection_times.clear()

# ============================================================================
# UART HANDLER - IMPROVED with buffer limit
# ============================================================================
class UARTHandler:
    """Handle UART with graceful degradation and buffer protection"""
    
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.packets_received = 0
        self.crc_failures = 0
        self.timeout_count = 0
        self.last_data = None
        self.emergency_stop_detected = False
        
        try:
            self._connect()
        except Exception as e:
            print(f"⚠ UART connection failed: {e}")
            print("  Continuing in camera-only mode...")
            self.connected = False
    
    def _connect(self):
        """Connect to UART with buffer flush"""
        self.serial_port = serial.Serial(
            UART_PORT,
            UART_BAUD,
            timeout=UART_TIMEOUT
        )
        
        # CRITICAL: Flush any stale data from previous session
        time.sleep(0.1)  # Let any in-flight bytes arrive
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        
        self.connected = True
        print(f"✓ UART connected: {UART_PORT} @ {UART_BAUD} baud")
        print(f"  Input/output buffers flushed")
    
    def read_teensy_data(self):
        """Read Teensy telemetry with buffer protection"""
        if not self.connected:
            return None
        
        try:
            # NEW: Bounded loop to prevent buffer overflow hang
            bytes_checked = 0
            
            while self.serial_port.in_waiting > 0 and bytes_checked < MAX_UART_BYTES_PER_READ:
                byte = self.serial_port.read(1)[0]
                bytes_checked += 1
                
                if byte == UART_STX:
                    length_byte = self.serial_port.read(1)
                    if len(length_byte) == 0:
                        self.timeout_count += 1
                        return self.last_data
                    
                    length = length_byte[0]
                    
                    # CRITICAL: Validate packet length BEFORE reading
                    if length > MAX_UART_PACKET_LENGTH or length < 10:
                        print(f"⚠ Invalid packet length: {length} bytes - skipping")
                        self.crc_failures += 1
                        continue  # Skip this packet, keep looking for next STX
                    
                    payload = self.serial_port.read(length)
                    if len(payload) != length:
                        self.timeout_count += 1
                        return self.last_data
                    
                    crc_bytes = self.serial_port.read(2)
                    if len(crc_bytes) != 2:
                        self.timeout_count += 1
                        return self.last_data
                    
                    received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
                    
                    etx_byte = self.serial_port.read(1)
                    if len(etx_byte) == 0 or etx_byte[0] != UART_ETX:
                        self.timeout_count += 1
                        return self.last_data
                    
                    calculated_crc = crc16_ccitt(payload)
                    
                    if calculated_crc == received_crc:
                        data_str = payload.decode('utf-8', errors='ignore')
                        self.packets_received += 1
                        self.last_data = data_str
                        
                        # Check for emergency stop
                        parts = data_str.split(',')
                        if len(parts) > 4:  # Use > for safety
                            try:
                                estop = int(parts[4])
                                if estop == 1 and not self.emergency_stop_detected:
                                    print("🛑 EMERGENCY STOP detected!")
                                    self.emergency_stop_detected = True
                                elif estop == 0 and self.emergency_stop_detected:
                                    print("✓ Emergency stop cleared")
                                    self.emergency_stop_detected = False
                            except (ValueError, IndexError):
                                pass
                        
                        return data_str
                    else:
                        self.crc_failures += 1
                        return self.last_data
            
            return self.last_data
        
        except serial.SerialException as e:
            print(f"⚠ UART error: {e}")
            self.connected = False
            return self.last_data
        except Exception as e:
            print(f"⚠ Unexpected UART error: {e}")
            return self.last_data
    
    def get_stats(self):
        """Return UART statistics"""
        if not self.connected:
            return "UART: Disconnected"
        
        total = self.packets_received + self.crc_failures
        if total == 0:
            return "UART: No data"
        
        success_rate = (self.packets_received / total) * 100
        estop_status = " | 🛑 E-STOP" if self.emergency_stop_detected else ""
        return f"UART: {success_rate:.1f}% valid | {self.crc_failures} CRC fails{estop_status}"
    
    def close(self):
        """Close serial port"""
        if self.serial_port and self.connected:
            try:
                self.serial_port.close()
                print("✓ UART closed")
            except Exception as e:
                print(f"⚠ UART close error: {e}")

# ============================================================================
# CSV LOGGER - IMPROVED with defensive parsing
# ============================================================================
class CSVLogger:
    """Log data to CSV with safe parsing"""
    
    def __init__(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = LOG_DIR / f'sensor_comparison_{timestamp}.csv'
        
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'frame_timestamp_ms',
            'teensy_timestamp_us',
            'camera_x_mm',
            'camera_y_mm',
            'detection_method',
            'touch_x_mm',
            'touch_y_mm',
            'ball_detected',
            'emergency_stop',
            'plate_cmd_x_deg',
            'plate_cmd_y_deg',
            'servo_actual_x_deg',
            'servo_actual_y_deg',
            'imu_roll_deg',
            'imu_pitch_deg',
            'imu_yaw_deg'
        ])
        
        self.row_count = 0
        print(f"✓ Logging to: {self.csv_path}")
    
    def log(self, teensy_data, camera_x, camera_y, detection_method):
        """Log with defensive parsing"""
        frame_timestamp_ms = int(time.time() * 1000)
        
        # Initialize with defaults
        teensy_timestamp_us = ''
        touch_x = ''
        touch_y = ''
        ball_detected = ''
        emergency_stop = ''
        plate_cmd_x = ''
        plate_cmd_y = ''
        servo_actual_x = ''
        servo_actual_y = ''
        imu_roll = ''
        imu_pitch = ''
        imu_yaw = ''
        
        # IMPROVED: Defensive parsing with explicit bounds
        if teensy_data:
            parts = teensy_data.split(',')
            
            # Basic fields (always present)
            teensy_timestamp_us = parts[0] if len(parts) > 0 else ''
            touch_x = parts[1] if len(parts) > 1 else ''
            touch_y = parts[2] if len(parts) > 2 else ''
            ball_detected = parts[3] if len(parts) > 3 else ''
            
            # Handle new format (with emergency_stop) vs old format
            if len(parts) >= 9:
                emergency_stop = parts[4]
                plate_cmd_x = parts[5]
                plate_cmd_y = parts[6]
                servo_actual_x = parts[7]
                servo_actual_y = parts[8]
                
                if len(parts) >= 12:
                    imu_roll = parts[9]
                    imu_pitch = parts[10]
                    imu_yaw = parts[11]
            elif len(parts) >= 8:  # Old format
                plate_cmd_x = parts[4]
                plate_cmd_y = parts[5]
                servo_actual_x = parts[6]
                servo_actual_y = parts[7]
                
                if len(parts) >= 11:
                    imu_roll = parts[8]
                    imu_pitch = parts[9]
                    imu_yaw = parts[10]
        
        self.csv_writer.writerow([
            frame_timestamp_ms,
            teensy_timestamp_us,
            camera_x if camera_x is not None else '',
            camera_y if camera_y is not None else '',
            detection_method if detection_method else '',
            touch_x,
            touch_y,
            ball_detected,
            emergency_stop,
            plate_cmd_x,
            plate_cmd_y,
            servo_actual_x,
            servo_actual_y,
            imu_roll,
            imu_pitch,
            imu_yaw
        ])
        
        self.row_count += 1
        
        if self.row_count >= MAX_LOG_ROWS:
            print(f"⚠ Max rows reached - rotating log")
            self.close()
            self.__init__()
    
    def close(self):
        """Close CSV file"""
        if self.csv_file:
            try:
                self.csv_file.close()
                print(f"✓ Log saved: {self.csv_path} ({self.row_count} rows)")
            except Exception as e:
                print(f"⚠ Log close error: {e}")

# ============================================================================
# FRAME SAVER - NEW non-blocking background thread
# ============================================================================
class FrameSaver:
    """Background thread for non-blocking frame saving"""
    
    def __init__(self):
        self.frame_queue = Queue(maxsize=10)
        self.thread = None
        self.running = False
        self.frames_saved = 0
        self.consecutive_failures = 0
        
        if SAVE_FRAMES:
            self.running = True
            self.thread = threading.Thread(target=self._worker, daemon=True)
            self.thread.start()
            print("✓ Frame saver thread started")
    
    def _worker(self):
        """Background worker for saving frames"""
        while self.running:
            try:
                frame_data = self.frame_queue.get(timeout=1.0)
                
                if frame_data is None:  # Shutdown signal
                    break
                
                frame_filename, display = frame_data
                
                # NEW: Check write success and detect filesystem issues
                try:
                    success = cv2.imwrite(str(frame_filename), display)
                    
                    if not success:
                        self.consecutive_failures += 1
                        if self.consecutive_failures >= 3:
                            print(f"⚠ CRITICAL: Frame saving failed 3 times - disk full?")
                    else:
                        self.consecutive_failures = 0
                        self.frames_saved += 1
                except Exception as e:
                    print(f"⚠ Frame write error: {e}")
                    self.consecutive_failures += 1
                
                self.frame_queue.task_done()
                
            except Exception:
                # Timeout or other error - continue
                pass
    
    def save_frame_async(self, frame_filename, display):
        """Non-blocking frame save - returns immediately"""
        if not SAVE_FRAMES or not self.running:
            return
        
        try:
            # Try to enqueue, skip if full (don't block main loop)
            self.frame_queue.put_nowait((frame_filename, display.copy()))
        except Full:
            pass  # Queue full, skip this frame
    
    def shutdown(self):
        """Shutdown frame saver thread"""
        if SAVE_FRAMES and self.running:
            self.running = False
            self.frame_queue.put(None)  # Signal shutdown
            if self.thread:
                self.thread.join(timeout=5.0)
            print(f"✓ Frame saver shutdown ({self.frames_saved} frames saved)")

# ============================================================================
# MAIN OBSERVER CLASS - IMPROVED
# ============================================================================
class SensorComparisonObserver:
    """Camera observer with non-blocking frame saving"""
    
    def __init__(self):
        print("Initializing Ball Balancer Observer...")
        print("=" * 70)
        
        self.detector = BallDetector()
        self.uart = UARTHandler()
        self.logger = CSVLogger()
        self.frame_saver = FrameSaver()  # NEW
        
        self.camera = self._init_camera()
        
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = time.time()
        
        # NEW: FPS measurement
        self.last_frame_time = None
        self.actual_fps_samples = deque(maxlen=100)
        self.fps_warning_shown = False
        
        # NEW: Latency measurement
        self.latency_samples = deque(maxlen=100)
        
        print()
        print("=" * 70)
        print("CRITICAL SAFETY FEATURES:")
        print("  ✓ Non-blocking frame saving (threading)")
        print("  ✓ UART buffer overflow protection (512 byte limit)")
        print("  ✓ Packet length validation (prevents hang)")
        print("  ✓ Serial buffer flush (prevents startup corruption)")
        print("  ✓ Actual FPS measurement and validation")
        print("  ✓ Control loop latency measurement")
        print("  ✓ Defensive parsing with bounds checking")
        print("  ✓ Safe camera cleanup")
        print("  ✓ Frame save error detection")
        print("=" * 70)
        print()
    
    def _init_camera(self):
        """Initialize camera with retry logic"""
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                camera = cv2.VideoCapture(CAMERA_INDEX)
                
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS_TARGET)
                
                camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                camera.set(cv2.CAP_PROP_EXPOSURE, -5)
                camera.set(cv2.CAP_PROP_AUTO_WB, 0)
                
                if not camera.isOpened():
                    raise RuntimeError("Camera failed to open")
                
                actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = camera.get(cv2.CAP_PROP_FPS)
                
                print(f"✓ Camera initialized (attempt {attempt+1})")
                print(f"  Resolution: {actual_width}×{actual_height}")
                print(f"  FPS: {actual_fps:.1f}")
                
                return camera
            
            except Exception as e:
                print(f"⚠ Camera init failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(1.0)
                else:
                    raise RuntimeError("Camera initialization failed")
    
    def run(self):
        """Main observation loop with FPS and latency measurement"""
        try:
            while True:
                frame_start = time.time()
                
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to read frame")
                    break
                
                # NEW: Measure actual FPS
                if self.last_frame_time is not None:
                    frame_interval = frame_start - self.last_frame_time
                    actual_fps = 1.0 / frame_interval if frame_interval > 0 else 0
                    self.actual_fps_samples.append(actual_fps)
                    
                    # Warn if significantly below target (only once)
                    if len(self.actual_fps_samples) >= 50 and not self.fps_warning_shown:
                        avg_fps = sum(self.actual_fps_samples) / len(self.actual_fps_samples)
                        
                        if avg_fps < CAMERA_FPS_TARGET * 0.8:
                            print(f"⚠ CRITICAL: Camera running at {avg_fps:.1f} FPS (target: {CAMERA_FPS_TARGET})")
                            print(f"  System timing assumptions may be invalid!")
                            print(f"  This will affect control loop performance.")
                            self.fps_warning_shown = True
                
                self.last_frame_time = frame_start
                
                # Detect ball
                ball_x_mm, ball_y_mm, ball_x_px, ball_y_px, radius_px, mask, method = \
                    self.detector.detect(frame)
                
                detected = (ball_x_mm is not None)
                
                # Read Teensy telemetry
                teensy_data = self.uart.read_teensy_data()
                
                # NEW: Measure control latency (approximate)
                if teensy_data and ball_x_mm is not None:
                    parts = teensy_data.split(',')
                    if len(parts) > 0:
                        try:
                            teensy_timestamp_us = int(parts[0])
                            pi_timestamp_us = int(time.time() * 1e6)
                            
                            # Latency = time difference (rough estimate due to unsynchronized clocks)
                            latency_ms = abs(pi_timestamp_us - teensy_timestamp_us) / 1000.0
                            
                            # Only consider reasonable values (clocks roughly synced within 1 second)
                            if latency_ms < 1000:
                                self.latency_samples.append(latency_ms)
                        except (ValueError, IndexError):
                            pass
                
                # Log
                self.logger.log(teensy_data, ball_x_mm, ball_y_mm, method)
                
                self.frame_count += 1
                if detected:
                    self.detection_count += 1
                
                # Create display
                display = self.create_display(frame, mask, ball_x_mm, ball_y_mm,
                                             ball_x_px, ball_y_px, radius_px,
                                             teensy_data, method)
                
                # IMPROVED: Non-blocking frame save
                if SAVE_FRAMES and (self.frame_count % FRAME_SAVE_INTERVAL == 0):
                    frame_filename = FRAME_SAVE_DIR / f'frame_{self.frame_count:06d}.jpg'
                    self.frame_saver.save_frame_async(frame_filename, display)
                
                cv2.imshow('Sensor Comparison Observer', display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.detector.reset_stats()
                    print("Statistics reset")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"\nERROR: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def get_actual_fps(self):
        """Get measured actual FPS"""
        if len(self.actual_fps_samples) == 0:
            return 0.0
        return sum(self.actual_fps_samples) / len(self.actual_fps_samples)
    
    def get_avg_latency(self):
        """Get average control loop latency"""
        if len(self.latency_samples) == 0:
            return 0.0
        return sum(self.latency_samples) / len(self.latency_samples)
    
    def create_display(self, frame, mask, ball_x_mm, ball_y_mm,
                      ball_x_px, ball_y_px, radius_px, teensy_data, method):
        """Create visualization with safe formatting"""
        display = frame.copy()
        
        center_x = CAMERA_WIDTH // 2
        center_y = CAMERA_HEIGHT // 2
        cv2.drawMarker(display, (center_x, center_y), (0, 255, 0),
                      cv2.MARKER_CROSS, 20, 2)
        
        # Draw detection - IMPROVED with safe formatting
        if ball_x_mm is not None and ball_x_px is not None:
            color = (0, 0, 255) if method == 'HSV' else (255, 0, 255)
            cv2.circle(display, (ball_x_px, ball_y_px), radius_px, color, 2)
            cv2.circle(display, (ball_x_px, ball_y_px), 3, color, -1)
            cv2.line(display, (center_x, center_y), (ball_x_px, ball_y_px),
                    (255, 255, 0), 1)
            
            # IMPROVED: Safe float formatting
            x_str = safe_format_float(ball_x_mm)
            y_str = safe_format_float(ball_y_mm)
            cv2.putText(display, f"Camera[{method}]: ({x_str}, {y_str}) mm",
                       (ball_x_px + 15, ball_y_px - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Stats
        runtime = time.time() - self.start_time
        fps = self.frame_count / runtime if runtime > 0 else 0
        detection_rate = (self.detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
        
        y_pos = 25
        cv2.putText(display, f"FPS: {fps:.1f}", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_pos += 25
        cv2.putText(display, f"Detection: {detection_rate:.1f}%", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_pos += 25
        cv2.putText(display, f"{self.detector.get_detection_stats()}", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_pos += 20
        
        avg_time = self.detector.get_avg_process_time()
        time_color = (0, 255, 0) if avg_time < DETECTION_TIME_WARNING_MS else (0, 165, 255)
        cv2.putText(display, f"Process: {avg_time:.1f} ms", (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, time_color, 1)
        
        # Teensy data
        if teensy_data:
            parts = teensy_data.split(',')
            if len(parts) > 2:  # Defensive check
                y_pos += 30
                touch_x = parts[1] if len(parts) > 1 else "N/A"
                touch_y = parts[2] if len(parts) > 2 else "N/A"
                cv2.putText(display, f"Touchscreen: ({touch_x}, {touch_y}) mm",
                           (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Sensor agreement
                if ball_x_mm is not None and len(parts) > 2:
                    try:
                        touch_x_f = float(parts[1])
                        touch_y_f = float(parts[2])
                        error = np.sqrt((touch_x_f - ball_x_mm)**2 + 
                                       (touch_y_f - ball_y_mm)**2)
                        y_pos += 20
                        cv2.putText(display, f"Sensor Error: {error:.1f} mm",
                                   (10, y_pos),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    except (ValueError, IndexError):
                        pass
        
        # UART stats
        y_pos = CAMERA_HEIGHT - 30
        uart_stats = self.uart.get_stats()
        uart_color = (0, 0, 255) if "E-STOP" in uart_stats else (255, 255, 255)
        cv2.putText(display, uart_stats, (10, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, uart_color, 1)
        
        # Mask
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_color, (160, 120))
        display[10:130, CAMERA_WIDTH-170:CAMERA_WIDTH-10] = mask_small
        
        return display
    
    def cleanup(self):
        """IMPROVED: Safe cleanup with exception handling"""
        print("\nShutting down...")
        
        # IMPROVED: Camera release with exception handling
        if self.camera:
            try:
                self.camera.release()
                print("✓ Camera released")
            except Exception as e:
                print(f"⚠ Camera release error: {e}")
        
        self.uart.close()
        self.logger.close()
        self.frame_saver.shutdown()  # NEW
        
        cv2.destroyAllWindows()
        
        runtime = time.time() - self.start_time
        fps = self.frame_count / runtime if runtime > 0 else 0
        detection_rate = (self.detection_count / self.frame_count * 100) if self.frame_count > 0 else 0
        
        print("=" * 70)
        print("Final Statistics:")
        print(f"  Runtime: {runtime:.1f} s")
        print(f"  Frames: {self.frame_count}")
        print(f"  Average FPS: {fps:.1f}")
        
        # NEW: Report actual measured FPS
        actual_fps = self.get_actual_fps()
        if actual_fps > 0:
            print(f"  Actual measured FPS: {actual_fps:.1f} (target was {CAMERA_FPS_TARGET})")
            if actual_fps < CAMERA_FPS_TARGET * 0.8:
                print(f"    ⚠ WARNING: Camera did not achieve target FPS!")
        
        print(f"  Detection rate: {detection_rate:.1f}%")
        print(f"  {self.detector.get_detection_stats()}")
        print(f"  Avg process time: {self.detector.get_avg_process_time():.1f} ms")
        
        # NEW: Report control loop latency
        avg_latency = self.get_avg_latency()
        if avg_latency > 0:
            print(f"  Avg control latency: {avg_latency:.1f} ms")
            if avg_latency > 50:
                print(f"    ⚠ WARNING: High latency detected!")
        
        print(f"  {self.uart.get_stats()}")
        if SAVE_FRAMES:
            print(f"  Frames saved: {self.frame_saver.frames_saved}")
        print("=" * 70)

# ============================================================================
# ENTRY POINT
# ============================================================================
def main():
    try:
        observer = SensorComparisonObserver()
        observer.run()
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
