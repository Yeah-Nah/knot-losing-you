# Connect the Rover to Wi‑Fi

This guide explains how to connect the rover to a known Wi‑Fi network for the first time using the built-in Wi‑Fi configuration tool. After this is done you will be able to SSH to the rover via the connected Wi-Fi.

> Source: Waveshare UGV Rover PI ROS2 documentation

## First step: power on the rover and join its hotspot

Before configuring the rover to connect to a known Wi‑Fi network, turn the rover on and let it finish booting.

Once it is powered on, check your phone or computer for the rover's Wi‑Fi hotspot. You can only connect to the rover hotspot one device at a time. The hotspot is usually named:

- `AccessPopup`

Connect to it using the default password:

- `1234567890`

After connecting, you can continue with the configuration steps below.

## Before you begin

When the rover starts for the first time, it normally runs in AP mode by default:

- Wi‑Fi name: `AccessPopup`
- Wi‑Fi password: `1234567890`
- Default IP in AP mode: `192.168.50.5`

If the rover is not connected to a known Wi‑Fi, it will create this hotspot automatically.

## Open JupyterLab

To configure the Wi‑Fi, open JupyterLab on the rover.

You can do this in either of these ways:

1. Open the main control page in a browser and click the JupyterLab button.
2. Open a browser and go to:
   - `http://192.168.50.5:8888`
   - or the robot's current IP address followed by `:8888`

## Step-by-step connection process

1. In the JupyterLab page, click the Terminal button at the bottom.

2. Enter the following command and press Enter to enter the project folder:

   ```bash
   bash
   ```

3. Change to the Wi‑Fi configuration tool folder:

   ```bash
   cd AccessPopup/
   ```

4. Make the configuration script executable:

   ```bash
   sudo chmod +x installconfig.sh
   ```

5. Run the script:

   ```bash
   sudo ./installconfig.sh
   ```

6. When the script opens, choose option `5` to set up a new Wi‑Fi connection.

7. The script will scan for nearby Wi‑Fi networks and display them with a number next to each one. Select the network you want to connect to by entering its number.

8. Enter the Wi‑Fi password when prompted.

9. After the rover connects successfully, the Wi‑Fi IP shown on the OLED display will change.

10. Open JupyterLab again using the new IP address shown on the OLED screen, because the rover's network address has changed.

11. When the script prompts you to continue, press any key.

12. To exit the Wi‑Fi configuration tool, press `9` and then Enter.

## Result

Once the rover has connected to the known Wi‑Fi, it runs in STA mode instead of AP mode.

You can then access the main control page in a browser using the robot's new IP address on port `5000`, for example:

```text
http://192.168.10.156:5000
```

## Notes

- If the rover moves out of range of that Wi‑Fi later, it can automatically create the default `AccessPopup` hotspot again.
- The rover and the device you are using must be on the same local network to access the web interface successfully.

## Related references

- Main project page: https://www.waveshare.com/wiki/UGV_Rover_PI_ROS2
