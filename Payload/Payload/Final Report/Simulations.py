#!/usr/bin/env python3
import cv2
import numpy as np
import time
import csv
import os
from datetime import datetime
from pathlib import Path
from collections import deque
import threading
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class RocketDynamics:
    def __init__(self, csv_file='VAYUVEGA_13_JUN_25_FINAL.csv'):
        self.start_time = time.time()
        self.interp_x = None
        self.interp_y = None
        self.duration = 0
        try:
            timestamps = []
            acc_x = []
            acc_y = []
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if not row: continue
                    try:
                        t_ms = float(row[0])
                        ax_raw = float(row[3])
                        ay_raw = float(row[5])
                        timestamps.append(t_ms)
                        acc_x.append(ax_raw)
                        acc_y.append(ay_raw)
                    except ValueError:
                        continue
            if not timestamps:
                self._set_fallback_interpolators()
                return
            times = np.array(timestamps)
            data_x = np.array(acc_x)
            data_y = np.array(acc_y)
            times = times / 1000.0
            times = times - times[0]
            self.duration = times[-1]
            data_x = data_x - np.mean(data_x)
            data_y = data_y - np.mean(data_y)
            self.interp_x = interp1d(times, data_x, kind='linear', bounds_error=False, fill_value=0.0)
            self.interp_y = interp1d(times, data_y, kind='linear', bounds_error=False, fill_value=0.0)
        except Exception:
            self._set_fallback_interpolators()

    def _set_fallback_interpolators(self):
        self.interp_x = lambda t: 0.0
        self.interp_y = lambda t: 0.0
        self.duration = 0.0

    def get_lateral_acceleration(self):
        sim_time = time.time() - self.start_time
        ax = float(self.interp_x(sim_time))
        ay = float(self.interp_y(sim_time))
        return ax, ay

class PhysicalBallPlate:
    def __init__(self, display_width=800, display_height=600):
        self.display_width = display_width
        self.display_height = display_height
        self.plate_width_m = 0.0767
        self.plate_height_m = 0.06365
        self.ball_radius_m = 0.005
        self.g_earth = 9.81
        self.a_rocket = 98.1
        self.g_eff_base = self.a_rocket + self.g_earth
        self.mu_s = 0.2
        self.FOS = 4.0
        self.theta_max_rad = np.arctan(self.mu_s * self.FOS)
        self.theta_max_deg = np.degrees(self.theta_max_rad)
        self.rocket = RocketDynamics()
        self.rolling_coeff = 5.0 / 7.0
        self.scale_x = display_width / self.plate_width_m
        self.scale_y = display_height / self.plate_height_m
        self.radius = int(self.ball_radius_m * min(self.scale_x, self.scale_y))
        self.x_m = 0.0
        self.y_m = 0.0
        self.vx_m = 0.0
        self.vy_m = 0.0
        self.tilt_x_actual = 0.0
        self.tilt_y_actual = 0.0
        self.disturb_ax = 0.0
        self.disturb_ay = 0.0
        self.dt = 1.0 / 240.0
        self.last_update = time.perf_counter()
        self.lock = threading.Lock()
        self.restitution = 0.7
        self.slip_warning_count = 0
        self.slip_warning_threshold = 0.9 * self.theta_max_deg
        self.reset()

    def reset(self):
        with self.lock:
            self.x_m = self.plate_width_m / 2.0
            self.y_m = self.plate_height_m / 2.0
            self.vx_m = 0.0
            self.vy_m = 0.0
            self.tilt_x_actual = 0.0
            self.tilt_y_actual = 0.0
            self.slip_warning_count = 0

    def _state_derivative(self, state, theta_x_rad, theta_y_rad, disturb_ax, disturb_ay):
        x, vx, y, vy = state
        tilt_accel_x = self.rolling_coeff * self.g_eff_base * np.sin(theta_x_rad)
        tilt_accel_y = self.rolling_coeff * self.g_eff_base * np.sin(theta_y_rad)
        ax = tilt_accel_x + disturb_ax
        ay = tilt_accel_y + disturb_ay
        return np.array([vx, ax, vy, ay])

    def update(self):
        current_time = time.perf_counter()
        dt = min(current_time - self.last_update, 0.033)
        self.last_update = current_time
        with self.lock:
            self.disturb_ax, self.disturb_ay = self.rocket.get_lateral_acceleration()
            theta_x_rad = np.deg2rad(self.tilt_x_actual)
            theta_y_rad = np.deg2rad(self.tilt_y_actual)
            state = np.array([self.x_m, self.vx_m, self.y_m, self.vy_m])
            k1 = self._state_derivative(state, theta_x_rad, theta_y_rad, self.disturb_ax, self.disturb_ay)
            k2 = self._state_derivative(state + 0.5 * dt * k1, theta_x_rad, theta_y_rad, self.disturb_ax, self.disturb_ay)
            k3 = self._state_derivative(state + 0.5 * dt * k2, theta_x_rad, theta_y_rad, self.disturb_ax, self.disturb_ay)
            k4 = self._state_derivative(state + dt * k3, theta_x_rad, theta_y_rad, self.disturb_ax, self.disturb_ay)
            state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
            self.x_m, self.vx_m, self.y_m, self.vy_m = state
            if self.x_m < self.ball_radius_m:
                self.x_m = self.ball_radius_m
                self.vx_m = -self.vx_m * self.restitution
            elif self.x_m > self.plate_width_m - self.ball_radius_m:
                self.x_m = self.plate_width_m - self.ball_radius_m
                self.vx_m = -self.vx_m * self.restitution
            if self.y_m < self.ball_radius_m:
                self.y_m = self.ball_radius_m
                self.vy_m = -self.vy_m * self.restitution
            elif self.y_m > self.plate_height_m - self.ball_radius_m:
                self.y_m = self.plate_height_m - self.ball_radius_m
                self.vy_m = -self.vy_m * self.restitution
        return self.get_position()

    def set_actual_tilt(self, tilt_x, tilt_y):
        with self.lock:
            self.tilt_x_actual = np.clip(tilt_x, -15, 15)
            self.tilt_y_actual = np.clip(tilt_y, -15, 15)

    def get_position(self):
        with self.lock:
            x_px = self.x_m * self.scale_x
            y_px = self.y_m * self.scale_y
            return float(x_px), float(y_px)

    def get_position_meters(self):
        with self.lock:
            return float(self.x_m), float(self.y_m)

    def get_velocity(self):
        with self.lock:
            vx_px = self.vx_m * self.scale_x
            vy_px = self.vy_m * self.scale_y
            return float(vx_px), float(vy_px)

    def get_velocity_meters(self):
        with self.lock:
            return float(self.vx_m), float(self.vy_m)

    def get_actual_tilt(self):
        with self.lock:
            return self.tilt_x_actual, self.tilt_y_actual

    def get_disturbances(self):
        with self.lock:
            return self.disturb_ax, self.disturb_ay

    def get_effective_gravity(self):
        return self.g_eff_base

    def get_friction_params(self):
        return self.mu_s, self.FOS, self.theta_max_deg

class ServoActuator:
    def __init__(self, tau=0.033, dt=1/240.0):
        self.tau = tau
        self.dt = dt
        self.bandwidth_hz = 1.0 / (2.0 * np.pi * self.tau)
        self.theta_min = -15.0
        self.theta_max = 15.0
        self.theta_x_cmd = 0.0
        self.theta_y_cmd = 0.0
        self.theta_x_actual = 0.0
        self.theta_y_actual = 0.0
        self.lock = threading.Lock()

    def set_command(self, tilt_x_cmd, tilt_y_cmd):
        with self.lock:
            self.theta_x_cmd = np.clip(tilt_x_cmd, self.theta_min, self.theta_max)
            self.theta_y_cmd = np.clip(tilt_y_cmd, self.theta_min, self.theta_max)

    def update(self):
        with self.lock:
            alpha = self.dt / self.tau
            self.theta_x_actual += alpha * (self.theta_x_cmd - self.theta_x_actual)
            self.theta_y_actual += alpha * (self.theta_y_cmd - self.theta_y_actual)
            self.theta_x_actual = np.clip(self.theta_x_actual, self.theta_min, self.theta_max)
            self.theta_y_actual = np.clip(self.theta_y_actual, self.theta_min, self.theta_max)

    def get_actual_angles(self):
        with self.lock:
            return self.theta_x_actual, self.theta_y_actual

    def get_commanded_angles(self):
        with self.lock:
            return self.theta_x_cmd, self.theta_y_cmd

    def get_tracking_error(self):
        with self.lock:
            error_x = self.theta_x_cmd - self.theta_x_actual
            error_y = self.theta_y_cmd - self.theta_y_actual
            return error_x, error_y

    def get_saturation_limits(self):
        return self.theta_min, self.theta_max

class FastTouchscreenSim:
    def __init__(self):
        self.screen_w = 320
        self.screen_h = 240
        self.update_rate = 100
        self.update_interval = 1.0 / self.update_rate
        self.last_update = 0
        self.total_latency = 0.26
        self.adc_min = 200
        self.adc_max = 3900
        self.cal_points_raw = [(240, 260), (3850, 280), (3820, 3870), (220, 3840)]
        self.cal_points_display = [(20, 20), (300, 20), (300, 220), (20, 220)]
        self._calculate_calibration_matrix()
        self.noise_std = 1.8
        self.history = deque(maxlen=10)

    def _calculate_calibration_matrix(self):
        src = np.array(self.cal_points_raw, dtype=np.float32)
        dst = np.array(self.cal_points_display, dtype=np.float32)
        self.cal_matrix = cv2.getPerspectiveTransform(src, dst)

    def _apply_calibration(self, raw_x, raw_y):
        point = np.array([[[raw_x, raw_y]]], dtype=np.float32)
        calibrated = cv2.perspectiveTransform(point, self.cal_matrix)
        return calibrated[0][0][0], calibrated[0][0][1]

    def read(self, world_x, world_y, world_w, world_h):
        now = time.perf_counter()
        true_x = (world_x / world_w) * self.screen_w
        true_y = (world_y / world_h) * self.screen_h
        self.history.append((now, true_x, true_y))
        if now - self.last_update < self.update_interval:
            if len(self.history) >= 2:
                _, lx, ly = self.history[-2]
                return int(lx), int(ly)
            return None, None
        self.last_update = now
        raw_x = int((true_x / self.screen_w) * (self.adc_max - self.adc_min) + self.adc_min)
        raw_y = int((true_y / self.screen_h) * (self.adc_max - self.adc_min) + self.adc_min)
        raw_x += int(np.random.normal(0, 10))
        raw_y += int(np.random.normal(0, 10))
        raw_x = np.clip(raw_x, self.adc_min, self.adc_max)
        raw_y = np.clip(raw_y, self.adc_min, self.adc_max)
        cal_x, cal_y = self._apply_calibration(raw_x, raw_y)
        cal_x += np.random.normal(0, self.noise_std)
        cal_y += np.random.normal(0, self.noise_std)
        cal_x = np.clip(cal_x, 0, self.screen_w)
        cal_y = np.clip(cal_y, 0, self.screen_h)
        return int(cal_x), int(cal_y)

class CoordinateMapper:
    def __init__(self, touch_w=320, touch_h=240, cam_w=800, cam_h=600):
        self.touch_w = touch_w
        self.touch_h = touch_h
        self.cam_w = cam_w
        self.cam_h = cam_h
        self.touch_points = np.array([[10, 10], [310, 10], [310, 230], [10, 230]], dtype=np.float32)
        self.camera_points = np.array([[25, 25], [775, 25], [775, 575], [25, 575]], dtype=np.float32)
        self.transform_matrix = cv2.getPerspectiveTransform(self.touch_points, self.camera_points)

    def touch_to_camera(self, touch_x, touch_y):
        point = np.array([[[touch_x, touch_y]]], dtype=np.float32)
        camera_point = cv2.perspectiveTransform(point, self.transform_matrix)
        return int(camera_point[0][0][0]), int(camera_point[0][0][1])

class PositionBasedController:
    def __init__(self, physics_system):
        self.physics = physics_system
        self.target_x_m = self.physics.plate_width_m / 2.0
        self.target_y_m = self.physics.plate_height_m / 2.0
        self.deadzone_radius_m = 0.003
        self.kp_x = 100.0
        self.kp_y = 100.0
        self.ki_x = 8.0
        self.ki_y = 8.0
        self.kd_x = 15.0
        self.kd_y = 15.0
        self.kff = 0.12
        self.kff_theory = (7.0/5.0) / self.physics.g_eff_base * (180.0/np.pi)
        self.kff_ratio = self.kff / self.kff_theory
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_limit = 2.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.last_update_time = time.perf_counter()

    def update(self):
        current_time = time.perf_counter()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        if dt <= 0:
            dt = 1.0 / 240.0
        x_m, y_m = self.physics.get_position_meters()
        vx_m, vy_m = self.physics.get_velocity_meters()
        disturb_ax, disturb_ay = self.physics.get_disturbances()
        error_x = self.target_x_m - x_m
        error_y = self.target_y_m - y_m
        error_dist = np.sqrt(error_x**2 + error_y**2)
        if error_dist < self.deadzone_radius_m:
            self.prev_error_x = error_x
            self.prev_error_y = error_y
            return 0.0, 0.0
        self.integral_x = np.clip(self.integral_x + error_x * dt, -self.integral_limit, self.integral_limit)
        self.integral_y = np.clip(self.integral_y + error_y * dt, -self.integral_limit, self.integral_limit)
        deriv_x = (error_x - self.prev_error_x) / dt
        deriv_y = (error_y - self.prev_error_y) / dt
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        tilt_x = (self.kp_x * error_x + self.ki_x * self.integral_x - self.kd_x * vx_m)
        tilt_y = (self.kp_y * error_y + self.ki_y * self.integral_y - self.kd_y * vy_m)
        tilt_x -= self.kff * disturb_ax
        tilt_y -= self.kff * disturb_ay
        return tilt_x, tilt_y

    def get_region_name(self):
        x_m, y_m = self.physics.get_position_meters()
        error_x_m = self.target_x_m - x_m
        error_y_m = self.target_y_m - y_m
        error_distance_mm = np.sqrt(error_x_m**2 + error_y_m**2) * 1000.0
        if error_distance_mm < 3.0:
            return "DEADZONE"
        elif error_distance_mm < 5.0:
            return "PRECISE"
        elif error_distance_mm < 10.0:
            return "ACTIVE"
        else:
            return "CORRECTING"

    def get_feedforward_info(self):
        return self.kff, self.kff_theory, self.kff_ratio

class DataLogger:
    def __init__(self, log_dir):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        self.time_data = []
        self.pos_x_mm = []
        self.pos_y_mm = []
        self.error_x_mm = []
        self.error_y_mm = []
        self.error_dist_mm = []
        self.tilt_x_cmd = []
        self.tilt_y_cmd = []
        self.tilt_x_actual = []
        self.tilt_y_actual = []
        self.servo_error_x = []
        self.servo_error_y = []
        self.disturb_ax = []
        self.disturb_ay = []
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    def log_sample(self, t, x_m, y_m, vx_m, vy_m, tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual, disturb_x, disturb_y, target_x_m, target_y_m):
        error_x_mm = (target_x_m - x_m) * 1000
        error_y_mm = (target_y_m - y_m) * 1000
        error_dist_mm = np.sqrt(error_x_mm**2 + error_y_mm**2)
        pos_x_mm = (x_m - target_x_m) * 1000
        pos_y_mm = (y_m - target_y_m) * 1000
        servo_error_x = tilt_x_cmd - tilt_x_actual
        servo_error_y = tilt_y_cmd - tilt_y_actual
        self.time_data.append(t)
        self.pos_x_mm.append(pos_x_mm)
        self.pos_y_mm.append(pos_y_mm)
        self.error_x_mm.append(error_x_mm)
        self.error_y_mm.append(error_y_mm)
        self.error_dist_mm.append(error_dist_mm)
        self.tilt_x_cmd.append(tilt_x_cmd)
        self.tilt_y_cmd.append(tilt_y_cmd)
        self.tilt_x_actual.append(tilt_x_actual)
        self.tilt_y_actual.append(tilt_y_actual)
        self.servo_error_x.append(servo_error_x)
        self.servo_error_y.append(servo_error_y)
        self.disturb_ax.append(disturb_x)
        self.disturb_ay.append(disturb_y)

    def generate_plots(self):
        if len(self.time_data) < 10:
            return
        t = np.array(self.time_data)
        
        plt.figure(figsize=(10, 6))
        plt.plot(t, self.error_x_mm, 'b-', label='Error X')
        plt.plot(t, self.error_y_mm, 'r-', label='Error Y')
        plt.axhline(y=3, color='g', linestyle='--', label='Deadzone')
        plt.axhline(y=-3, color='g', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (mm)')
        plt.title('Position Error')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_position_error_{self.timestamp}.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.plot(t, self.tilt_x_cmd, 'r--', label='Cmd X')
        plt.plot(t, self.tilt_x_actual, 'b-', label='Act X')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (deg)')
        plt.title('Tilt Response X')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_tilt_x_{self.timestamp}.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.plot(t, self.tilt_y_cmd, 'r--', label='Cmd Y')
        plt.plot(t, self.tilt_y_actual, 'b-', label='Act Y')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (deg)')
        plt.title('Tilt Response Y')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_tilt_y_{self.timestamp}.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.plot(t, self.servo_error_x, 'b-', label='Err X')
        plt.plot(t, self.servo_error_y, 'r-', label='Err Y')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (deg)')
        plt.title('Servo Tracking Error')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_servo_error_{self.timestamp}.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.plot(t, self.disturb_ax, 'orange', label='Dist X')
        plt.plot(t, self.disturb_ay, 'brown', label='Dist Y')
        plt.xlabel('Time (s)')
        plt.ylabel('Acc (m/s²)')
        plt.title('Disturbances')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_disturbances_{self.timestamp}.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.plot(t, [-p for p in self.pos_x_mm], 'b-', label='Pos X (inv)')
        plt.plot(t, self.tilt_x_actual, 'r-', label='Tilt X')
        plt.xlabel('Time (s)')
        plt.title('Correlation X')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_corr_x_{self.timestamp}.png')
        plt.close()

        plt.figure(figsize=(10, 6))
        plt.plot(t, [-p for p in self.pos_y_mm], 'b-', label='Pos Y (inv)')
        plt.plot(t, self.tilt_y_actual, 'r-', label='Tilt Y')
        plt.xlabel('Time (s)')
        plt.title('Correlation Y')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.log_dir / f'plot_corr_y_{self.timestamp}.png')
        plt.close()

class Camera2DRenderer:
    def __init__(self, width=800, height=600):
        self.width = width
        self.height = height
        self.frame_buffer = np.zeros((height, width), dtype=np.uint8)

    def render(self, ball_x, ball_y, ball_r):
        self.frame_buffer.fill(200)
        cv2.circle(self.frame_buffer, (int(ball_x), int(ball_y)), int(ball_r), 25, -1, cv2.LINE_AA)
        noise = np.random.normal(0, 1.2, self.frame_buffer.shape)
        self.frame_buffer = np.clip(self.frame_buffer.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return self.frame_buffer

class BlobDetector:
    def __init__(self):
        self.threshold = 120
        self.min_area = 200
        self.max_area = 3000
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.detection_times = deque(maxlen=60)
        self.last_detections = deque(maxlen=3)
        self.capture_latency = 8.33
        self.processing_latency = 0

    def detect(self, frame, capture_timestamp):
        proc_start = time.perf_counter()
        _, binary = cv2.threshold(frame, self.threshold, 255, cv2.THRESH_BINARY_INV)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel, iterations=1)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_x, best_y, best_r = None, None, None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    if area > best_area:
                        best_area = area
                        best_x = int(M["m10"] / M["m00"])
                        best_y = int(M["m01"] / M["m00"])
                        best_r = int(np.sqrt(area / np.pi))
        if best_x is not None:
            self.last_detections.append((best_x, best_y, best_r))
            if len(self.last_detections) >= 2:
                xs = [d[0] for d in self.last_detections]
                ys = [d[1] for d in self.last_detections]
                best_x = int(np.median(xs))
                best_y = int(np.median(ys))
        self.processing_latency = (time.perf_counter() - proc_start) * 1000
        self.detection_times.append(self.processing_latency)
        return best_x, best_y, best_r, binary

    def get_total_latency(self):
        return self.capture_latency + self.processing_latency

    def get_avg_processing(self):
        return np.mean(self.detection_times) if self.detection_times else 0

class Platform3DRenderer:
    def __init__(self, width=500, height=500):
        self.width = width
        self.height = height
        self.view_angle = 35
        self.view_rotation = 45
        self.platform_w = 300
        self.platform_h = 300
        self.platform_thickness = 20
        self.bg_color = (240, 240, 245)
        self.platform_color = (220, 220, 230)
        self.platform_edge = (100, 100, 120)
        self.grid_color = (180, 180, 200)
        self.ball_color = (30, 30, 200)
        self.ball_shadow = (150, 150, 150)

    def project_3d(self, x, y, z):
        angle_rad = np.radians(self.view_rotation)
        x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad)
        y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad)
        scale = 1.0 / (1.0 + z * 0.001)
        screen_x = x_rot * scale + self.width / 2
        screen_y = (y_rot * 0.5 - z) * scale + self.height * 0.6
        return int(screen_x), int(screen_y)

    def render(self, ball_x, ball_y, ball_r, tilt_x, tilt_y):
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = self.bg_color
        plat_x = (ball_x / 800) * self.platform_w - self.platform_w/2
        plat_y = (ball_y / 600) * self.platform_h - self.platform_h/2
        tilt_x_rad = np.radians(tilt_x)
        tilt_y_rad = np.radians(tilt_y)
        corners = [
            (-self.platform_w/2, -self.platform_h/2),
            (self.platform_w/2, -self.platform_h/2),
            (self.platform_w/2, self.platform_h/2),
            (-self.platform_w/2, self.platform_h/2),
        ]
        corners_3d = []
        for cx, cy in corners:
            z_tilt = cx * np.sin(tilt_y_rad) + cy * np.sin(tilt_x_rad)
            corners_3d.append((cx, cy, z_tilt))
        bottom_points = []
        for cx, cy, cz in corners_3d:
            px, py = self.project_3d(cx, cy, cz - self.platform_thickness)
            bottom_points.append([px, py])
        bottom_points = np.array(bottom_points, dtype=np.int32)
        cv2.fillPoly(frame, [bottom_points], (180, 180, 190))
        top_points = []
        for cx, cy, cz in corners_3d:
            px, py = self.project_3d(cx, cy, cz)
            top_points.append([px, py])
        top_points = np.array(top_points, dtype=np.int32)
        cv2.fillPoly(frame, [top_points], self.platform_color)
        for i in range(-3, 4):
            x_pos = i * self.platform_w / 6
            p1 = self.project_3d(x_pos, -self.platform_h/2, x_pos * np.sin(tilt_y_rad))
            p2 = self.project_3d(x_pos, self.platform_h/2, x_pos * np.sin(tilt_y_rad))
            cv2.line(frame, p1, p2, self.grid_color, 1)
            y_pos = i * self.platform_h / 6
            p1 = self.project_3d(-self.platform_w/2, y_pos, y_pos * np.sin(tilt_x_rad))
            p2 = self.project_3d(self.platform_w/2, y_pos, y_pos * np.sin(tilt_x_rad))
            cv2.line(frame, p1, p2, self.grid_color, 1)
        cv2.polylines(frame, [top_points], True, self.platform_edge, 3)
        for i in range(4):
            side_points = np.array([
                top_points[i], top_points[(i+1)%4],
                bottom_points[(i+1)%4], bottom_points[i]
            ], dtype=np.int32)
            cv2.fillPoly(frame, [side_points], (200, 200, 210))
            cv2.polylines(frame, [side_points], True, self.platform_edge, 1)
        ball_z = plat_x * np.sin(tilt_y_rad) + plat_y * np.sin(tilt_x_rad)
        shadow_x, shadow_y = self.project_3d(plat_x, plat_y, ball_z)
        ball_r_3d = int(ball_r * 0.6)
        cv2.ellipse(frame, (shadow_x, shadow_y + 5), (ball_r_3d, ball_r_3d//3), 0, 0, 360, self.ball_shadow, -1)
        ball_screen_x, ball_screen_y = self.project_3d(plat_x, plat_y, ball_z + ball_r_3d)
        for r in range(ball_r_3d, 0, -2):
            intensity = 0.4 + (r / ball_r_3d) * 0.6
            color = tuple(int(c * intensity) for c in self.ball_color)
            cv2.circle(frame, (ball_screen_x, ball_screen_y), r, color, -1)
        cv2.circle(frame, (ball_screen_x - ball_r_3d//3, ball_screen_y - ball_r_3d//3), ball_r_3d // 4, (100, 100, 255), -1)
        return frame

class OptimizedSimulation:
    def __init__(self, target_fps=60):
        self.target_fps = target_fps
        self.physics = PhysicalBallPlate(800, 600)
        self.servo = ServoActuator(tau=0.033, dt=1/240.0)
        self.controller = PositionBasedController(self.physics)
        self.platform_3d = Platform3DRenderer(500, 500)
        self.camera_2d = Camera2DRenderer(800, 600)
        self.detector = BlobDetector()
        self.touchscreen = FastTouchscreenSim()
        self.coord_mapper = CoordinateMapper()
        self.log_dir = Path("simulation_logs")
        self.log_dir.mkdir(exist_ok=True)
        self.data_logger = DataLogger(self.log_dir)
        self.last_log_time = 0
        self.log_interval = 0.05
        self.control_on = True
        self.show_help = True
        self.paused = False
        self.frame_count = 0
        self.fps_history = deque(maxlen=60)
        self.cam_detects = 0
        self.total_frames = 0
        self.errors = deque(maxlen=100)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = self.log_dir / f'rocket_payload_{ts}.csv'
        self._init_csv()
        self.frames_dir = Path("frames")
        self.frames_dir.mkdir(exist_ok=True)
        self.session_dir = self.frames_dir / f"session_{ts}"
        self.session_dir.mkdir(exist_ok=True)
        self.video_file = self.log_dir / f"simulation_{ts}.mp4"
        self.frame_save_count = 0
        self.physics_running = True
        self.physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.physics_thread.start()
        self.sim_start_time = time.time()

    def _init_csv(self):
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'sim_time_s', 'pos_x_mm', 'pos_y_mm', 'error_x_mm', 'error_y_mm', 'error_dist_mm',
                'tilt_x_cmd', 'tilt_y_cmd', 'tilt_x_actual', 'tilt_y_actual',
                'servo_error_x', 'servo_error_y', 'disturb_ax', 'disturb_ay',
                'region', 'velocity_ms'
            ])

    def _physics_loop(self):
        while self.physics_running:
            start = time.perf_counter()
            tilt_x_actual, tilt_y_actual = self.servo.get_actual_angles()
            self.physics.set_actual_tilt(tilt_x_actual, tilt_y_actual)
            ball_x, ball_y = self.physics.update()
            if self.control_on:
                tilt_x_cmd, tilt_y_cmd = self.controller.update()
                self.servo.set_command(tilt_x_cmd, tilt_y_cmd)
            else:
                self.servo.set_command(0.0, 0.0)
            self.servo.update()
            elapsed = time.perf_counter() - start
            sleep = max(0, (1.0/240.0) - elapsed)
            if sleep > 0:
                time.sleep(sleep)

    def run(self):
        cv2.namedWindow('Rocket Payload Simulator', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Rocket Payload Simulator', 1600, 600)
        try:
            while True:
                loop_start = time.perf_counter()
                sim_time = time.time() - self.sim_start_time
                if not self.paused:
                    ball_x, ball_y = self.physics.get_position()
                    tilt_x_cmd, tilt_y_cmd = self.servo.get_commanded_angles()
                    tilt_x_actual, tilt_y_actual = self.servo.get_actual_angles()
                    disturb_ax, disturb_ay = self.physics.get_disturbances()
                    cam_frame = self.camera_2d.render(ball_x, ball_y, self.physics.radius)
                    capture_time = time.perf_counter()
                    cam_x, cam_y, cam_r, cam_mask = self.detector.detect(cam_frame, capture_time)
                    touch_x, touch_y = self.touchscreen.read(ball_x, ball_y, 800, 600)
                    touch_cam_x, touch_cam_y = None, None
                    if touch_x:
                        touch_cam_x, touch_cam_y = self.coord_mapper.touch_to_camera(touch_x, touch_y)
                    error = None
                    if cam_x and touch_cam_x:
                        error = np.sqrt((cam_x - touch_cam_x)**2 + (cam_y - touch_cam_y)**2)
                        self.errors.append(error)
                    region = self.controller.get_region_name()
                    self.total_frames += 1
                    if cam_x:
                        self.cam_detects += 1
                    self._log_csv(sim_time, region, tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual, disturb_ax, disturb_ay)
                    if sim_time - self.last_log_time >= self.log_interval:
                        x_m, y_m = self.physics.get_position_meters()
                        vx_m, vy_m = self.physics.get_velocity_meters()
                        self.data_logger.log_sample(
                            sim_time, x_m, y_m, vx_m, vy_m,
                            tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual,
                            disturb_ax, disturb_ay,
                            self.controller.target_x_m, self.controller.target_y_m
                        )
                        self.last_log_time = sim_time
                    display = self._create_display(
                        cam_frame, cam_mask, ball_x, ball_y, self.physics.radius,
                        cam_x, cam_y, cam_r, touch_x, touch_y,
                        touch_cam_x, touch_cam_y, error, tilt_x_cmd, tilt_y_cmd,
                        tilt_x_actual, tilt_y_actual, region, disturb_ax, disturb_ay
                    )
                    frame_filename = self.session_dir / f"frame_{self.frame_save_count:06d}.png"
                    cv2.imwrite(str(frame_filename), display)
                    self.frame_save_count += 1
                    cv2.imshow('Rocket Payload Simulator', display)
                    self.frame_count += 1
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    break
                elif key == ord(' '):
                    self.physics.reset()
                    self.servo.theta_x_actual = 0.0
                    self.servo.theta_y_actual = 0.0
                    self.servo.theta_x_cmd = 0.0
                    self.servo.theta_y_cmd = 0.0
                elif key == ord('c'):
                    self.control_on = not self.control_on
                elif key == ord('h'):
                    self.show_help = not self.show_help
                elif key == ord('p'):
                    self.paused = not self.paused
                elapsed = time.perf_counter() - loop_start
                if elapsed > 0:
                    self.fps_history.append(1.0 / elapsed)
                sleep = (1.0/self.target_fps) - elapsed
                if sleep > 0:
                    time.sleep(sleep)
        finally:
            self.physics_running = False
            cv2.destroyAllWindows()
            self._print_stats()
            self.data_logger.generate_plots()
            if self.frame_save_count > 0:
                self._create_video_from_frames()

    def _create_display(self, cam_frame, cam_mask, ball_x, ball_y, ball_r, cam_x, cam_y, cam_r_det, touch_x, touch_y, touch_cam_x, touch_cam_y, error, tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual, region, disturb_ax, disturb_ay):
        display_h = 600
        display_w = 1600
        display = np.ones((display_h, display_w, 3), dtype=np.uint8) * 245
        platform_3d = self.platform_3d.render(ball_x, ball_y, ball_r, tilt_x_actual, tilt_y_actual)
        display[50:550, 50:550] = platform_3d
        cam_color = cv2.cvtColor(cam_frame, cv2.COLOR_GRAY2BGR)
        cv2.circle(cam_color, (int(ball_x), int(ball_y)), int(ball_r), (0, 255, 0), 2)
        if cam_x:
            cv2.circle(cam_color, (cam_x, cam_y), cam_r_det, (255, 200, 0), 2)
        if touch_cam_x:
            cv2.drawMarker(cam_color, (touch_cam_x, touch_cam_y), (0, 150, 255), cv2.MARKER_CROSS, 25, 2)
            if cam_x:
                cv2.line(cam_color, (cam_x, cam_y), (touch_cam_x, touch_cam_y), (255, 255, 255), 1)
        cam_small = cv2.resize(cam_color, (480, 360))
        display[50:410, 600:1080] = cam_small
        mask_color = cv2.cvtColor(cam_mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_color, (240, 180))
        display[50:230, 1120:1360] = mask_small
        cv2.rectangle(display, (1118, 48), (1362, 232), (100, 100, 120), 2)
        self._draw_stats(display, error, region, disturb_ax, disturb_ay, tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual, 1100, 250)
        self._draw_legend(display, 600, 420)
        if self.show_help:
            self._draw_help(display, 50, display_h - 100)
        return display

    def _draw_stats(self, display, error, region, disturb_ax, disturb_ay, tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual, x, y):
        cv2.rectangle(display, (x, y), (x+450, y+340), (255, 255, 255), -1)
        cv2.rectangle(display, (x, y), (x+450, y+340), (100, 100, 120), 2)
        fps = np.mean(self.fps_history) if self.fps_history else 0
        det_rate = (self.cam_detects / self.total_frames * 100) if self.total_frames else 0
        vx, vy = self.physics.get_velocity()
        speed = np.sqrt(vx**2 + vy**2)
        x_m, y_m = self.physics.get_position_meters()
        error_x_mm = (self.controller.target_x_m - x_m) * 1000.0
        error_y_mm = (self.controller.target_y_m - y_m) * 1000.0
        servo_err_x = tilt_x_cmd - tilt_x_actual
        servo_err_y = tilt_y_cmd - tilt_y_actual
        theta_mag = np.sqrt(tilt_x_actual**2 + tilt_y_actual**2)
        mu_s, FOS, theta_max = self.physics.get_friction_params()
        slip_margin = theta_max / theta_mag if theta_mag > 0.1 else 999
        log_samples = len(self.data_logger.time_data)
        lines = [
            f"FPS: {fps:.1f}",
            f"Detection: {det_rate:.1f}%",
            f"Frames saved: {self.frame_save_count}",
            f"Logged: {log_samples} samples",
            "",
            "Control Region: " + region,
            "",
            "Position Error:",
            f"  X: {error_x_mm:+.2f} mm",
            f"  Y: {error_y_mm:+.2f} mm",
            "",
            "Servo (Cmd -> Act):",
            f"  X: {tilt_x_cmd:+.2f} -> {tilt_x_actual:+.2f}",
            f"  Y: {tilt_y_cmd:+.2f} -> {tilt_y_actual:+.2f}",
            f"  Err: {servo_err_x:+.3f}, {servo_err_y:+.3f}",
            "",
            f"Slip margin: {slip_margin:.2f}x",
            f"Speed: {speed:.1f} px/s",
        ]
        line_y = y + 50
        for text in lines:
            cv2.putText(display, text, (x+15, line_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (60, 60, 80), 1)
            line_y += 18

    def _draw_legend(self, display, x, y):
        cv2.rectangle(display, (x, y), (x+450, y+70), (255, 255, 255), -1)
        cv2.rectangle(display, (x, y), (x+450, y+70), (100, 100, 120), 2)
        legends = [
            (x+20, "True Position", (0, 255, 0)),
            (x+170, "Camera Detect", (255, 200, 0)),
            (x+320, "Touchscreen", (0, 150, 255)),
        ]
        for lx, text, color in legends:
            cv2.circle(display, (lx, y+45), 6, color, -1)
            cv2.putText(display, text, (lx+15, y+50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (60, 60, 80), 1)

    def _draw_help(self, display, x, y):
        cv2.rectangle(display, (x, y), (x+1100, y+50), (255, 255, 255), -1)
        cv2.rectangle(display, (x, y), (x+1100, y+50), (100, 100, 120), 2)
        help_text = "SPACE=Reset | C=Control ON/OFF | H=Hide Help | P=Pause | Q=Quit (generates plots)"
        cv2.putText(display, help_text, (x+10, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 80), 1)

    def _log_csv(self, sim_time, region, tilt_x_cmd, tilt_y_cmd, tilt_x_actual, tilt_y_actual, disturb_ax, disturb_ay):
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            x_m, y_m = self.physics.get_position_meters()
            vx_m, vy_m = self.physics.get_velocity_meters()
            pos_x_mm = (x_m - self.controller.target_x_m) * 1000.0
            pos_y_mm = (y_m - self.controller.target_y_m) * 1000.0
            error_x_mm = (self.controller.target_x_m - x_m) * 1000.0
            error_y_mm = (self.controller.target_y_m - y_m) * 1000.0
            error_dist_mm = np.sqrt(error_x_mm**2 + error_y_mm**2)
            speed = np.sqrt(vx_m**2 + vy_m**2)
            servo_err_x = tilt_x_cmd - tilt_x_actual
            servo_err_y = tilt_y_cmd - tilt_y_actual
            writer.writerow([
                f'{sim_time:.3f}', f'{pos_x_mm:.3f}', f'{pos_y_mm:.3f}', f'{error_x_mm:.3f}', f'{error_y_mm:.3f}', f'{error_dist_mm:.3f}',
                f'{tilt_x_cmd:.3f}', f'{tilt_y_cmd:.3f}', f'{tilt_x_actual:.3f}', f'{tilt_y_actual:.3f}',
                f'{servo_err_x:.3f}', f'{servo_err_y:.3f}', f'{disturb_ax:.4f}', f'{disturb_ay:.4f}',
                region, f'{speed:.4f}'
            ])

    def _create_video_from_frames(self):
        try:
            first_frame_path = self.session_dir / "frame_000000.png"
            first_frame = cv2.imread(str(first_frame_path))
            if first_frame is None:
                return
            height, width = first_frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(str(self.video_file), fourcc, self.target_fps, (width, height))
            for i in range(self.frame_save_count):
                frame_path = self.session_dir / f"frame_{i:06d}.png"
                frame = cv2.imread(str(frame_path))
                if frame is not None:
                    video_writer.write(frame)
            video_writer.release()
        except Exception:
            pass

    def _print_stats(self):
        if len(self.data_logger.time_data) > 100:
            ss_start = int(0.7 * len(self.data_logger.time_data))
            err_x = np.array(self.data_logger.error_x_mm[ss_start:])
            err_y = np.array(self.data_logger.error_y_mm[ss_start:])
            servo_err_x = np.array(self.data_logger.servo_error_x[ss_start:])
            servo_err_y = np.array(self.data_logger.servo_error_y[ss_start:])
            rms_err_x = np.sqrt(np.mean(err_x**2))
            rms_err_y = np.sqrt(np.mean(err_y**2))
            rms_servo_x = np.sqrt(np.mean(servo_err_x**2))
            rms_servo_y = np.sqrt(np.mean(servo_err_y**2))
            pos_x_ss = np.array(self.data_logger.pos_x_mm[ss_start:])
            pos_y_ss = np.array(self.data_logger.pos_y_mm[ss_start:])
            tilt_x_ss = np.array(self.data_logger.tilt_x_actual[ss_start:])
            tilt_y_ss = np.array(self.data_logger.tilt_y_actual[ss_start:])
            corr_x = np.corrcoef(-pos_x_ss, tilt_x_ss)[0, 1] if len(pos_x_ss) > 1 else 0
            corr_y = np.corrcoef(-pos_y_ss, tilt_y_ss)[0, 1] if len(pos_y_ss) > 1 else 0
            print("Table 1: Representative steady-state metrics")
            print("Metric                              X axis    Y axis")
            print(f"RMS position error (mm)             {rms_err_x:.2f}      {rms_err_y:.2f}")
            print(f"RMS servo tracking error (deg)      {rms_servo_x:.3f}     {rms_servo_y:.3f}")
            print(f"Position–tilt correlation           {corr_x:.3f}     {corr_y:.3f}")

def _ensure_csv_exists():
    filename = "VAYUVEGA_13_JUN_25_FINAL.csv"
    if not os.path.exists(filename):
        content = """Timestamp,Pressure,Altitude_P,AccX,AccY,AccZ,Lat,Lon,Altitude_GPS,State
-13921,911.62,-2.39,-0.9,10.24,1.22,0,0,0,0
-53,911.41,-0.34,0.71,160.55,5.22,0,0,0,0
0,911.53,-1.51,14.67,83.16,-26.83,0,0,0,1
27,910.29,10.6,2.79,159.18,-3.06,0,0,0,1
54,912.27,-8.73,-0.51,160.44,5.88,0,0,0,1
111,911.04,3.27,0.43,160.36,-5.37,0,0,0,1
137,910.79,5.72,-5.37,159.93,-5.26,0,0,0,1
164,911.04,3.27,-0.43,160.59,-5.06,0,0,0,1
445,909.18,21.46,1.02,160.59,-1.1,0,0,0,1
608,907.57,37.22,0.04,160.59,-1.22,0,0,0,1
826,904.84,64.01,-2.08,160.59,1.57,0,0,0,1
1101,900.75,104.26,1.18,160.59,-2.59,0,0,0,1
1658,891.77,193.15,2.75,160.59,-2.79,0,0,0,1
2261,875.85,352.54,17.53,160.59,-4.12,0,0,0,1
2931,848.2,635.03,29.54,109.87,-1.88,0,0,0,1
3457,810.7,1030.32,11.06,3.41,9.81,0,0,0,1
"""
        with open(filename, 'w') as f:
            f.write(content)

def main():
    _ensure_csv_exists()
    sim = OptimizedSimulation(target_fps=60)
    sim.run()

if __name__ == '__main__':
    main()