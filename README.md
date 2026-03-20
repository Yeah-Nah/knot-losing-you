# knot-losing-you
Autonomous UGV that follows you around the boat park using OAK-D Lite and LiDAR. No sea legs required.

### Technical Specifications for This Project
- Raspberry Pi 5
- Waveshare UGV Rover
- OAKd Lite

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

# Create an venv and activate
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
