# knot-losing-you
Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR. No sea legs required.

### Technical Specifications for This Project
- Raspberry Pi 5
- Waveshare UGV Rover
- OAK-D Lite

### Development Machine

From the repo root:

```bash
# Change directory to the module
cd /ugv-follower

# Install the package
pip install -e ".[dev]"
```

### Pi Machine

```bash
# Clone the repo
git clone https://github.com/Yeah-Nah/knot-losing-you.git
cd knot-losing-you/ugv-follower

# Create a venv and activate
cd ~/knot-losing-you/ugv-follower # Working directory must be /ugv-follower
python3 -m venv .venv
source .venv/bin/activate

# Install the project
pip install -e .

# Activate non-root access to OAKd camera (optional)
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### For testing rover sensors and cameras
```bash
# For checking the battery level
python check_battery.py

# For testing the camera access. Will return log of frames received.
python test_camera_access.py

# For testing LiDAR connection. Will return breakdown of packet information received.
python test_lidar_access.py

# For testing motor controls. Will drive foward 0.2m in 1s.
python test_ugv_controller.py
```
