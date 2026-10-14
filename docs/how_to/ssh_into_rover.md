# SSH Into the Rover from Windows PowerShell

This guide explains how to SSH into the rover after the rover has already been connected to your Wi-Fi network.

Use Windows PowerShell for these steps.

## Before you begin

Make sure the rover is powered on and connected to the same Wi-Fi network as your Windows computer.

You also need the rover's current Wi-Fi IP address. The rover shows this IP address on its OLED display after it connects to Wi-Fi.

Example rover IP address:

```text
192.168.10.156
```

## Open Windows PowerShell

1. On your Windows computer, press the Windows key.
2. Type `PowerShell`.
3. Click `Windows PowerShell`.

Do not use the browser, JupyterLab terminal, or Command Prompt for these instructions. Use Windows PowerShell on your Windows computer.

## SSH into the rover

In Windows PowerShell, run this command, replacing `192.168.10.156` with the IP address shown on the rover's OLED display:

```powershell
ssh ws@192.168.10.156
```
if that doesn't work try
```powershell
ssh pi@192.168.10.156
```

When PowerShell asks whether you want to continue connecting, type:

```text
yes
```

Then press Enter.

When PowerShell asks for the password, enter the rover password and press Enter.

```text
ws
```

The password characters will not appear while you type. This is normal.

## Confirm you are connected

After logging in successfully, the prompt should change to something that looks like this:

```text
pi@raspberrypi:~ $
```

You are now running commands on the rover, not on your Windows computer.

## Exit the SSH session

When you are finished, run:

```bash
exit
```

This closes the SSH session and returns PowerShell to your Windows computer.

## Troubleshooting

- If PowerShell says `ssh` is not recognized, install the Windows OpenSSH Client from Windows Optional Features.
- If the connection times out, make sure your Windows computer and the rover are on the same Wi-Fi network.
- If the connection is refused, wait for the rover to finish booting and try again.
- If the IP address does not work, check the OLED display again. The rover's IP address can change when it reconnects to Wi-Fi.