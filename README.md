# Basler Camera Calibration Tool

A simple Python tool for monocular camera calibration using Basler cameras and OpenCV.

## Features

- Automatic Basler camera discovery and selection
- Interactive camera parameter configuration (FPS, gain, exposure)
- Real-time video display with checkerboard corner detection
- Easy-to-use TUI for capturing calibration images
- OpenCV-based camera calibration
- ROS-compatible YAML output format
- Reprojection error calculation

## Requirements

- Python 3.14+
- Basler camera with Pylon drivers installed
- Checkerboard calibration pattern (default: 9x6 inner corners, 25mm squares)

## Installation

The project uses `uv` for dependency management. Dependencies are already configured in `pyproject.toml`:

```bash
uv sync
```

## Usage

Run the calibration script:

```bash
uv run python calibrate.py
```

Or make it executable:

```bash
chmod +x calibrate.py
./calibrate.py
```

### Calibration Process

1. **Camera Selection**: The tool will detect all available Basler cameras and prompt you to select one

2. **Parameter Configuration**: Configure camera parameters:
   - Frame rate (FPS)
   - Gain (dB)
   - Exposure time (microseconds)
   - Checkerboard dimensions (inner corners)
   - Square size (meters)

3. **Calibration Capture**:
   - A live video window will open showing the camera feed
   - Move your checkerboard pattern in front of the camera
   - When corners are detected, they will be highlighted in the video
   - Press `SPACE` to capture an image when corners are detected
   - Try to capture images from different angles and distances (at least 10-15 images recommended)

4. **Keyboard Controls**:
   - `SPACE`: Capture current frame (when corners are detected)
   - `r`: Reset and clear all captured images
   - `q` or `ESC`: Finish capture and compute calibration

5. **Results**:
   - Calibration will be computed automatically
   - Results saved to `calibrations/camera_calibration_YYYYMMDD_HHMMSS.yaml`
   - Reprojection error displayed and saved in the YAML file

## Output Format

The calibration results are saved in ROS-compatible YAML format with the following information:

- Camera matrix (intrinsic parameters)
- Distortion coefficients (plumb_bob model)
- Image dimensions
- Rectification matrix
- Projection matrix
- Reprojection error (as a comment at the top)

## Checkerboard Pattern

The default configuration expects a checkerboard with:
- 9x6 inner corners (10x7 squares)
- 25mm (0.025m) square size

You can modify these parameters during the configuration step or by editing the script defaults in `calibrate.py:22-23`.

## Tips for Good Calibration

1. Capture at least 10-15 images (more is better)
2. Cover different areas of the image
3. Include images at different angles and orientations
4. Include images at different distances from the camera
5. Ensure the entire checkerboard is visible and well-lit
6. Keep the checkerboard flat and rigid
7. Aim for a reprojection error < 0.5 pixels

## Troubleshooting

**No cameras found**: Ensure Pylon drivers are installed and cameras are connected

**Checkerboard not detected**:
- Improve lighting conditions
- Ensure checkerboard is flat and fully visible
- Verify checkerboard dimensions match configuration
- Try adjusting exposure settings

**High reprojection error**:
- Capture more images
- Ensure checkerboard is flat
- Recalibrate with better image coverage
- Check that square size is correctly specified
