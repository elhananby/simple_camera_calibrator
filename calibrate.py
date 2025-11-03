#!/usr/bin/env python3
"""
Simple monocular camera calibration tool for Basler cameras.
"""

import cv2
import numpy as np
from pypylon import pylon
import sys
from datetime import datetime
from pathlib import Path


class CameraCalibrator:
    def __init__(self):
        self.camera = None
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        # Calibration data
        self.object_points = []  # 3D points in real world space
        self.image_points = []   # 2D points in image plane
        self.image_size = None

        # Checkerboard configuration (can be modified)
        self.checkerboard_size = (9, 6)  # inner corners
        self.square_size = 0.025  # 25mm squares

        # Camera parameters
        self.fps = 30
        self.gain = 0
        self.exposure = 10000  # microseconds

    def discover_cameras(self):
        """Discover all available Basler cameras."""
        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()

        if len(devices) == 0:
            print("No Basler cameras found!")
            sys.exit(1)

        return devices

    def select_camera(self, devices):
        """Display available cameras and let user select one."""
        print("\n" + "="*60)
        print("Available Basler Cameras:")
        print("="*60)

        for idx, device in enumerate(devices):
            print(f"{idx + 1}. {device.GetFriendlyName()}")
            print(f"   Model: {device.GetModelName()}")
            print(f"   Serial: {device.GetSerialNumber()}")
            print()

        while True:
            try:
                choice = input(f"Select camera (1-{len(devices)}): ").strip()
                idx = int(choice) - 1
                if 0 <= idx < len(devices):
                    return devices[idx]
                else:
                    print(f"Please enter a number between 1 and {len(devices)}")
            except (ValueError, KeyboardInterrupt):
                print("\nExiting...")
                sys.exit(0)

    def configure_camera_params(self):
        """Allow user to configure camera parameters."""
        print("\n" + "="*60)
        print("Camera Parameters Configuration")
        print("="*60)
        print("Press Enter to use default values shown in brackets")
        print()

        # FPS
        fps_input = input(f"Frame rate (FPS) [{self.fps}]: ").strip()
        if fps_input:
            try:
                self.fps = float(fps_input)
            except ValueError:
                print(f"Invalid value, using default: {self.fps}")

        # Gain
        gain_input = input(f"Gain (dB) [{self.gain}]: ").strip()
        if gain_input:
            try:
                self.gain = float(gain_input)
            except ValueError:
                print(f"Invalid value, using default: {self.gain}")

        # Exposure
        exposure_input = input(f"Exposure (microseconds) [{self.exposure}]: ").strip()
        if exposure_input:
            try:
                self.exposure = int(exposure_input)
            except ValueError:
                print(f"Invalid value, using default: {self.exposure}")

        # Checkerboard parameters
        print("\n" + "-"*60)
        print("Checkerboard Configuration")
        print("-"*60)

        cols_input = input(f"Checkerboard columns (inner corners) [{self.checkerboard_size[0]}]: ").strip()
        if cols_input:
            try:
                cols = int(cols_input)
                rows_input = input(f"Checkerboard rows (inner corners) [{self.checkerboard_size[1]}]: ").strip()
                rows = int(rows_input) if rows_input else self.checkerboard_size[1]
                self.checkerboard_size = (cols, rows)
            except ValueError:
                print(f"Invalid value, using default: {self.checkerboard_size}")

        square_input = input(f"Square size (meters) [{self.square_size}]: ").strip()
        if square_input:
            try:
                self.square_size = float(square_input)
            except ValueError:
                print(f"Invalid value, using default: {self.square_size}")

        print("\nConfiguration complete!")
        print(f"FPS: {self.fps}, Gain: {self.gain} dB, Exposure: {self.exposure} µs")
        print(f"Checkerboard: {self.checkerboard_size[0]}x{self.checkerboard_size[1]}, Square: {self.square_size}m")
        print()

    def initialize_camera(self, device):
        """Initialize and configure the selected camera."""
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(device))
        self.camera.Open()

        # Configure camera parameters
        try:
            # Set acquisition frame rate
            self.camera.AcquisitionFrameRateEnable.SetValue(True)
            self.camera.AcquisitionFrameRate.SetValue(self.fps)

            # Set gain
            if self.camera.Gain.IsWritable():
                self.camera.Gain.SetValue(self.gain)

            # Set exposure
            if self.camera.ExposureTime.IsWritable():
                self.camera.ExposureTime.SetValue(self.exposure)

            print(f"Camera configured: {self.fps} FPS, {self.gain} dB gain, {self.exposure} µs exposure")

        except Exception as e:
            print(f"Warning: Some parameters could not be set: {e}")

        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def prepare_object_points(self):
        """Prepare 3D object points for the checkerboard."""
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard_size[0],
                                0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp

    def run_calibration(self):
        """Main calibration loop with live video display."""
        print("\n" + "="*60)
        print("Camera Calibration")
        print("="*60)
        print("Instructions:")
        print("  - Move checkerboard in front of the camera")
        print("  - Press SPACE to capture an image when corners are detected")
        print("  - Press 'q' or ESC to finish and compute calibration")
        print("  - Press 'r' to reset and clear all captured images")
        print("="*60)
        print()

        objp = self.prepare_object_points()
        captured_count = 0

        try:
            while self.camera.IsGrabbing():
                grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                if grab_result.GrabSucceeded():
                    # Convert to OpenCV format
                    image = self.converter.Convert(grab_result)
                    img = image.GetArray()

                    if self.image_size is None:
                        self.image_size = (img.shape[1], img.shape[0])

                    # Convert to grayscale for corner detection
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    # Find checkerboard corners
                    ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

                    # Draw corners if found
                    display_img = img.copy()
                    if ret:
                        # Refine corner positions
                        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                        cv2.drawChessboardCorners(display_img, self.checkerboard_size, corners_refined, ret)

                        # Display instruction
                        cv2.putText(display_img, "Press SPACE to capture", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(display_img, "Checkerboard not detected", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    # Display capture count
                    cv2.putText(display_img, f"Captured: {captured_count}", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    cv2.imshow('Camera Calibration', display_img)

                    # Handle keyboard input
                    key = cv2.waitKey(1) & 0xFF

                    if key == ord(' ') and ret:  # Space bar - capture
                        self.object_points.append(objp)
                        self.image_points.append(corners_refined)
                        captured_count += 1
                        print(f"Captured image {captured_count}")

                        # Flash feedback
                        flash = np.ones_like(display_img) * 255
                        cv2.imshow('Camera Calibration', flash)
                        cv2.waitKey(100)

                    elif key == ord('r'):  # Reset
                        self.object_points = []
                        self.image_points = []
                        captured_count = 0
                        print("Reset: All captured images cleared")

                    elif key == ord('q') or key == 27:  # q or ESC - quit
                        break

                grab_result.Release()

        except KeyboardInterrupt:
            print("\nInterrupted by user")

        finally:
            cv2.destroyAllWindows()
            self.camera.StopGrabbing()
            self.camera.Close()

        return captured_count

    def compute_calibration(self):
        """Compute camera calibration from collected points."""
        if len(self.object_points) < 3:
            print(f"\nError: Need at least 3 calibration images, only {len(self.object_points)} captured")
            return None

        print(f"\nComputing calibration from {len(self.object_points)} images...")

        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, self.image_size, None, None
        )

        # Calculate reprojection error
        mean_error = 0
        for i in range(len(self.object_points)):
            imgpoints2, _ = cv2.projectPoints(self.object_points[i], rvecs[i], tvecs[i],
                                             camera_matrix, dist_coeffs)
            error = cv2.norm(self.image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        mean_error /= len(self.object_points)

        print(f"Calibration successful!")
        print(f"Reprojection error: {mean_error:.4f} pixels")

        return {
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            'reprojection_error': mean_error,
            'image_size': self.image_size
        }

    def save_calibration(self, calibration_data):
        """Save calibration in ROS-compatible YAML format."""
        if calibration_data is None:
            return

        # Create calibrations directory if it doesn't exist
        calib_dir = Path("calibrations")
        calib_dir.mkdir(exist_ok=True)

        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = calib_dir / f"camera_calibration_{timestamp}.yaml"

        camera_matrix = calibration_data['camera_matrix']
        dist_coeffs = calibration_data['dist_coeffs']
        image_size = calibration_data['image_size']
        reprojection_error = calibration_data['reprojection_error']

        # Format in ROS camera_calibration format
        yaml_content = f"""# Camera calibration results
# Reprojection error: {reprojection_error:.6f} pixels
# Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
# Number of images used: {len(self.object_points)}
# Checkerboard: {self.checkerboard_size[0]}x{self.checkerboard_size[1]}, square size: {self.square_size}m

image_width: {image_size[0]}
image_height: {image_size[1]}
camera_name: basler_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [{', '.join(f'{x:.6f}' for x in camera_matrix.flatten())}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {len(dist_coeffs[0])}
  data: [{', '.join(f'{x:.6f}' for x in dist_coeffs.flatten())}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{camera_matrix[0,0]:.6f}, 0.0, {camera_matrix[0,2]:.6f}, 0.0, 0.0, {camera_matrix[1,1]:.6f}, {camera_matrix[1,2]:.6f}, 0.0, 0.0, 0.0, 1.0, 0.0]
"""

        with open(filename, 'w') as f:
            f.write(yaml_content)

        print(f"\nCalibration saved to: {filename}")
        print(f"Image size: {image_size[0]}x{image_size[1]}")
        print(f"Focal length: fx={camera_matrix[0,0]:.2f}, fy={camera_matrix[1,1]:.2f}")
        print(f"Principal point: cx={camera_matrix[0,2]:.2f}, cy={camera_matrix[1,2]:.2f}")
        print(f"Distortion coefficients: {dist_coeffs.flatten()}")


def main():
    print("="*60)
    print("Basler Camera Calibration Tool")
    print("="*60)

    calibrator = CameraCalibrator()

    # Discover cameras
    devices = calibrator.discover_cameras()

    # Select camera
    selected_device = calibrator.select_camera(devices)

    # Configure parameters
    calibrator.configure_camera_params()

    # Initialize camera
    calibrator.initialize_camera(selected_device)

    # Run calibration
    captured_count = calibrator.run_calibration()

    # Compute and save calibration
    if captured_count >= 3:
        calibration_data = calibrator.compute_calibration()
        calibrator.save_calibration(calibration_data)
    else:
        print("\nCalibration cancelled: Not enough images captured")


if __name__ == "__main__":
    main()
