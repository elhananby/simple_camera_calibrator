# Basler Camera Calibration Tool

A simple Python tool for monocular camera calibration using Basler cameras and OpenCV.

## Features

- Automatic Basler camera discovery and selection
- TOML-based configuration for camera and calibration parameters
- Auto-capture mode with quality-based image selection
- Real-time video display with checkerboard corner detection
- Image quality scoring based on sharpness metrics
- Manual capture override available at any time
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

## Configuration

Edit `config.toml` to customize calibration parameters:

```toml
[camera]
fps = 10.0
gain = 0.0  # dB
exposure = 10000  # microseconds

[checkerboard]
columns = 8  # inner corners
rows = 6  # inner corners
square_size = 0.025  # meters

[capture]
auto_capture = true  # enable automatic capture
capture_interval = 1.0  # seconds between captures
max_images = 0  # 0 = unlimited
min_quality = 0.8  # quality threshold (0.0-1.0)
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

2. **Auto-Capture Mode** (default):
   - A live video window will open showing the camera feed
   - Move your checkerboard pattern in front of the camera
   - When corners are detected, the quality score is displayed
   - Images are automatically captured when:
     - Checkerboard corners are detected
     - Quality score meets the threshold (default: 0.8)
     - Minimum time interval has passed (default: 1.0s)
   - A white flash indicates successful capture
   - Try to capture images from different angles and distances (at least 10-15 images recommended)

3. **Keyboard Controls**:
   - `SPACE`: Manually capture current frame (always available, bypasses quality checks)
   - `r`: Reset and clear all captured images
   - `q` or `ESC`: Finish capture and compute calibration

4. **Results**:
   - Calibration will be computed automatically (requires minimum 3 images)
   - Results saved to `results/Basler-[serial].yaml`
   - Reprojection error displayed and saved in the YAML file

## Manual Capture Mode

To disable auto-capture and use manual capture only, edit `config.toml`:

```toml
[capture]
auto_capture = false
```

In manual mode:
- Press `SPACE` to capture images when checkerboard is detected
- Quality score is still displayed to help you assess image sharpness
- Full control over when images are captured

## Output Format

The calibration results are saved in ROS-compatible YAML format with the following information:

- Camera matrix (intrinsic parameters)
- Distortion coefficients (plumb_bob model)
- Image dimensions
- Rectification matrix
- Projection matrix
- Reprojection error (as a comment at the top)
- Number of images used
- Checkerboard configuration

## Checkerboard Pattern

The default configuration expects a checkerboard with:
- 8x6 inner corners (9x7 squares)
- 25mm (0.025m) square size

You can modify these parameters in the `config.toml` file.

## Tips for Good Calibration

1. **Image Quantity**: Capture at least 10-15 images (more is better)
2. **Coverage**: Cover different areas of the image sensor
3. **Variety**: Include images at different angles and orientations
4. **Distance**: Include images at different distances from the camera
5. **Visibility**: Ensure the entire checkerboard is visible and well-lit
6. **Pattern Quality**: Keep the checkerboard flat and rigid
7. **Quality Threshold**: Adjust `min_quality` in config.toml if auto-capture is too strict or too lenient
   - Higher values (0.7-0.9) = stricter, sharper images only
   - Lower values (0.3-0.6) = more lenient, accepts slightly blurry images
8. **Target Error**: Aim for a reprojection error < 0.5 pixels

## Troubleshooting

**No cameras found**:
- Ensure Pylon drivers are installed and cameras are connected
- Check USB connection and power

**Checkerboard not detected**:
- Improve lighting conditions
- Ensure checkerboard is flat and fully visible
- Verify checkerboard dimensions match configuration in `config.toml`
- Try adjusting exposure settings in `config.toml`

**Auto-capture not triggering**:
- Check quality score shown in video feed
- Lower `min_quality` threshold in `config.toml` (try 0.5-0.6)
- Improve lighting or focus to increase sharpness
- Use manual capture with SPACE bar as alternative

**Images captured too quickly/slowly**:
- Adjust `capture_interval` in `config.toml`
- Set `max_images` to limit total captures

**High reprojection error**:
- Capture more images with better variety
- Ensure checkerboard is flat and rigid
- Increase `min_quality` to capture only sharp images
- Verify `square_size` in `config.toml` matches your physical checkerboard
- Recalibrate with better coverage of the image sensor
