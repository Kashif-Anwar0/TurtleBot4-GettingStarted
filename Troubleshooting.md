# TurtleBot 4  - Troubleshooting Guide

This guide provides detailed solutions for common issues encountered during TurtleBot 4 setup and operation. Work through the relevant sections based on the problem you are facing.

**General Tips:**
*   **Reboot:** Often the first step. Reboot both the robot (`sudo reboot` via SSH or power cycle Create 3) and potentially your workstation.
*   **Check Connections:** Physically verify all cables, especially the USB-C between the Pi and Create 3, power cables, and sensor cables. Reseat them firmly.
*   **Check Power:** Ensure the Create 3 battery is charged and properly seated. Check if the charging dock is powered and making contact.
*   **Check SD Card:** SD cards can corrupt easily. Try re-flashing, using a different reliable flashing tool, or using a different known-good SD card. Verify the flash if possible.
*   **Check ROS Domain ID:** Ensure `echo $ROS_DOMAIN_ID` is consistent between the robot and workstation if running nodes across the network (usually `0` by default, but can cause issues if mismatched). Consider CycloneDDS configuration for robust discovery ([Clearpath Docs often cover this](https://docs.clearpathrobotics.com/docs/robots/turtlebot4/networking)).
*   **Logs:** Use `journalctl`, `dmesg`, and ROS 2 logging to find specific error messages. Example: `journalctl -u NetworkManager -n 50 --no-pager` or check logs in `~/.ros/log`.

---

## Table of Contents

*   [A. Flashing & Boot Issues](#a-flashing--boot-issues)
*   [B. Cannot Connect/SSH in AP Mode (10.42.0.1)](#b-cannot-connectssh-in-ap-mode-104201)
*   [C. Netplan Permission / Apply Issues](#c-netplan-permission--apply-issues)
*   [D. Cannot Run `turtlebot4_setup` Tool](#d-cannot-run-turtlebot4_setup-tool)
*   [E. WiFi Client Mode Fails (Cannot connect to your WiFi)](#e-wifi-client-mode-fails-cannot-connect-to-your-wifi)
*   [F. Cannot Connect/SSH in Client Mode (New IP)](#f-cannot-connectssh-in-client-mode-new-ip)
*   [G. ROS 2 Nodes/Topics Not Working](#g-ros-2-nodestopics-not-working)
*   [H. Controller/Teleop Not Working](#h-controllerteleop-not-working)
*   [I. Specific Hardware Issues](#i-specific-hardware-issues)

---

### A. Flashing & Boot Issues

**Symptom:** Robot doesn't power on Pi, screen stays blank, boot sequence seems stuck, flashing tool reports errors.

1.  **Flashing Errors:**
    *   **Cause:** Bad SD card, faulty reader, incomplete download, wrong image file.
    *   **Solution:** Use a high-quality SD card. Try a different USB port/reader. Re-download the image and verify checksum if available. Use Raspberry Pi Imager or BalenaEtcher. Ensure you selected the correct drive. Try formatting the SD card first.
2.  **No Power to Pi / Blank Screen (after Create 3 powers on):**
    *   **Cause:** Loose power cable from Create 3 to Pi, faulty cable, Pi hardware issue, severely corrupted bootloader/firmware on SD card (bad flash).
    *   **Solution:** Check/reseat the power cable. Try a different known-good SD card flashed with a basic Raspberry Pi OS to rule out Pi hardware failure. Check Create 3 battery level.
3.  **Stuck During Boot (Screen shows logo/text but doesn't reach AP mode):**
    *   **Cause:** Filesystem corruption on SD card, incomplete first boot setup, hardware conflict, ROS service failing critically early.
    *   **Solution:** Re-flash the SD card. If persistent, try a different SD card. Connect keyboard/monitor directly to Pi to observe boot messages for specific errors (requires removing robot casing). Check `dmesg` or boot logs if accessible.

---

### B. Cannot Connect/SSH in AP Mode (10.42.0.1)

**Symptom:** Robot screen shows `IP: 10.42.0.1`, `Turtlebot4` WiFi visible, but `ping 10.42.0.1` fails or `ssh ubuntu@10.42.0.1` times out/refused.

1.  **Workstation Not Connected Correctly:**
    *   **Cause:** Workstation connected to wrong WiFi, password typo (`Turtlebot4`), workstation network issue.
    *   **Solution:** Disconnect/reconnect workstation to `Turtlebot4` WiFi. Verify password. Try disabling/re-enabling WiFi on workstation. Try from a different computer/phone.
2.  **Robot Network Service Issue:**
    *   **Cause:** `hostapd` (AP service) or `dhcpd` (IP assignment) failed to start correctly on the Pi. Filesystem issue preventing service start.
    *   **Solution:** Reboot robot. Re-flash SD card. Connect keyboard/monitor to check service status (`systemctl status hostapd`, `systemctl status dnsmasq` or similar DHCP server). Check logs (`journalctl`).
3.  **Firewall Issues (Less Common):**
    *   **Cause:** Firewall on workstation blocking outgoing SSH/ICMP, or (very unlikely) firewall on Pi misconfigured.
    *   **Solution:** Temporarily disable workstation firewall for testing. Re-flash SD card to reset Pi firewall.
4.  **Hardware:**
    *   **Cause:** Faulty Pi WiFi module, faulty USB-C connection between Pi and Create 3 (can sometimes cause system instability).
    *   **Solution:** Check USB-C cable. Test Pi WiFi by trying to connect it to another network via direct keyboard/monitor access if possible. Consider Pi replacement if WiFi is faulty.

---

### C. Netplan Permission / Apply Issues

**Symptom:** Warnings about `/etc/netplan/*.yaml` permissions being "too open". `sudo netplan apply` fails with "Operation not permitted" or other errors.

1.  **Incorrect Permissions:**
    *   **Cause:** File permissions were changed from the secure default (owner read/write only).
    *   **Solution:** SSH into robot and run:
        ```bash
        sudo chmod 600 /etc/netplan/40-ethernets.yaml
        sudo chmod 600 /etc/netplan/50-wifis.yaml
        ls -l /etc/netplan # Verify permissions are -rw------- root root
        ```
2.  **`sudo netplan apply` Fails ("Operation not permitted", other errors):**
    *   **Cause:** Often occurs after a previous `netplan apply` failed mid-process (e.g., due to one file having bad permissions), leaving network state inconsistent or lockfiles present. Filesystem might have remounted read-only.
    *   **Solution:**
        *   **Reboot:** `sudo reboot`. This resolves most temporary lock/state issues. After reboot, SSH back in and try `sudo netplan apply` again.
        *   **Check Mounts:** If reboot doesn't fix, check filesystem: `mount | grep " / "`. Ensure it shows `rw` (read-write), not `ro` (read-only). If `ro`, filesystem corruption likely -> Re-flash SD card.
        *   **Check Netplan/NetworkManager Logs:** `sudo netplan --debug apply`, `journalctl -u NetworkManager -n 50`. Look for specific errors.

---

### D. Cannot Run `turtlebot4_setup` Tool

**Symptom:** `ros2 run turtlebot4_setup turtlebot4_setup` fails with errors like "package not found", "executable not found", or Python tracebacks.

1.  **ROS 2 Environment Not Sourced:**
    *   **Cause:** The SSH session doesn't know where the ROS 2 installation is.
    *   **Solution:** Run `source /opt/ros/jazzy/setup.bash` in the SSH terminal, then try the `ros2 run` command again. Verify with `echo $ROS_DISTRO` (should output `jazzy`). Add the source command to `~/.bashrc` for persistence: `echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc` and start a new SSH session.
2.  **Package Not Installed / Corrupted:**
    *   **Cause:** Image flash was incomplete, filesystem corruption, package removed accidentally.
    *   **Solution:**
        *   Verify package exists: `ros2 pkg list | grep turtlebot4_setup` (should show the package).
        *   Verify executable exists: `ros2 pkg executables turtlebot4_setup` (should list `turtlebot4_setup`).
        *   If missing -> Re-flash the SD card with the official TurtleBot 4 image. Attempting manual installation (`sudo apt install ros-jazzy-turtlebot4-setup`) might work but suggests a deeper problem with the base image.
3.  **Python/Dependency Issues:**
    *   **Cause:** Underlying Python environment issue, missing dependencies.
    *   **Solution:** Examine the Python traceback for specific errors (e.g., `ImportError`). Try reinstalling related packages (`sudo apt update && sudo apt --reinstall install ros-jazzy-turtlebot4-setup` and potentially related dependencies). Often simpler to re-flash.

---

### E. WiFi Client Mode Fails (Cannot connect to your WiFi)

**Symptom:** After saving client settings in `turtlebot4_setup`, screen shows `IP: UNKNOWN` or similar error, `Turtlebot4` AP may or may not reappear, robot doesn't get an IP from your router.

1.  **Incorrect SSID/Password:**
    *   **Cause:** Typo, case-sensitivity error, wrong security type implicitly assumed.
    *   **Solution:** Power cycle the robot (wait 10s off). Hope it reverts to AP mode. If it does, SSH in (`10.42.0.1`) and run `turtlebot4_setup` again. **Triple-check** SSID and password. Use copy-paste if possible. Be mindful of special characters.
2.  **Weak WiFi Signal:**
    *   **Cause:** Robot too far from router/AP.
    *   **Solution:** Move robot closer to the router for initial setup. Consider WiFi extenders if needed later.
3.  **Router Issues:**
    *   **Cause:** MAC Address Filtering enabled on router (blocking the Pi), DHCP server issue (no IPs available or server down), router needs reboot, unsupported security standard (e.g., WPA3 issues if Pi doesn't support it fully, though unlikely with Pi 4). 2.4GHz vs 5GHz band issues (Pi 4 supports both, but ensure SSID is compatible).
    *   **Solution:** Check router admin page: disable MAC filtering temporarily, check DHCP pool/leases, reboot router. Try connecting another new device to the WiFi to confirm the network is okay. Ensure you are connecting to a 2.4GHz or compatible 5GHz network.
4.  **NetworkManager Confusion:**
    *   **Cause:** Corrupted NetworkManager profiles from previous attempts or prior user.
    *   **Solution:** (Advanced) If robot reverts to AP mode, SSH in. Use `nmcli connection show` to list profiles. `sudo nmcli connection delete <profile_name_or_uuid>` to remove old/failed profiles for your home WiFi. Then try `turtlebot4_setup` again. **Be careful** not to delete essential profiles. Re-flashing is often safer.
5.  **Robot Stuck (Doesn't revert to AP):**
    *   **Cause:** Network configuration is corrupted mid-transition.
    *   **Solution:** Re-flash the SD card. This is often the quickest way to reset the network state cleanly.

---

### F. Cannot Connect/SSH in Client Mode (New IP)

**Symptom:** Robot screen shows a valid IP address from your network, but you cannot ping or SSH into it from your workstation (which is on the same network).

1.  **Workstation on Different Network:**
    *   **Cause:** Workstation connected to a different WiFi (e.g., guest network, 5GHz vs 2.4GHz band if they are separate networks) or Ethernet.
    *   **Solution:** Ensure workstation is on the **exact same** WiFi network SSID as the robot.
2.  **Incorrect IP Address:**
    *   **Cause:** Misread screen, IP changed (DHCP lease expired), typo.
    *   **Solution:** Double-check IP on robot screen. Re-scan network or check router's device list for the current IP of "ubuntu" or "turtlebot4".
3.  **Firewall:**
    *   **Cause:** Workstation firewall blocking outgoing SSH/ICMP. Router firewall rules blocking intra-network traffic (less common on home networks).
    *   **Solution:** Temporarily disable workstation firewall. Check router settings for "AP Isolation" or "Client Isolation" and disable it.
4.  **Robot SSH Service Issue:**
    *   **Cause:** `sshd` service failed to start after network change.
    *   **Solution:** Reboot robot. If persistent, connect keyboard/monitor to check `systemctl status sshd` and `journalctl -u sshd`. Re-flash if unresolved.
5.  **Network Discovery Issues (ROS 2 specific):**
    *   **Cause:** While SSH might work, ROS 2 nodes might not see each other due to DDS discovery issues across networks (less likely if SSH works but worth noting).
    *   **Solution:** Check `ROS_DOMAIN_ID`. Configure CycloneDDS XML file for reliable discovery if needed (see Clearpath docs).

---

### G. ROS 2 Nodes/Topics Not Working

**Symptom:** SSH works, but `ros2 node list` is empty/missing key nodes (like `/create3_node`, `/lidar_node`), or `ros2 topic echo` shows no data.

1.  **Core Services Not Running:**
    *   **Cause:** The main `turtlebot4.service` (or similar systemd unit) failed to start or crashed.
    *   **Solution:** Check service status: `systemctl status turtlebot4.service`. Check logs: `journalctl -u turtlebot4.service -n 100 --no-pager`. Try restarting: `sudo systemctl restart turtlebot4.service`. Look for specific errors in the logs (e.g., sensor connection failed, configuration file missing).
2.  **Create 3 Communication Issue:**
    *   **Cause:** Loose/faulty USB-C cable between Pi and Create 3. Create 3 base is off, in a fault state, or its firmware has issues.
    *   **Solution:** Check/reseat USB-C cable firmly at both ends. Ensure Create 3 is powered on (check lights/sounds). Check `/diagnostics` topic for Create 3 errors. Power cycle Create 3. Check iRobot Create 3 documentation for its status indicators.
3.  **Sensor Issues:**
    *   **Cause:** Sensor unplugged, faulty, or driver crashed.
    *   **Solution:** Check physical connection of LiDAR, Camera. Check logs for related errors (e.g., `lidar_node`, `camera_node`). Check `/diagnostics`. If a specific sensor node is missing/crashing, suspect hardware failure of that sensor.
4.  **Resource Issues:**
    *   **Cause:** Pi running out of memory or CPU, causing nodes to crash. Filesystem full.
    *   **Solution:** Check `htop` or `top` via SSH for resource usage. Check disk space `df -h`. Close unnecessary processes. Consider if custom code added is too demanding.
5.  **Configuration Errors:**
    *   **Cause:** Incorrect parameters in launch files or YAML configuration files used by the TurtleBot 4 nodes.
    *   **Solution:** If you modified config files, revert changes. Check default config files in `/opt/ros/jazzy/share/turtlebot4...` packages. Re-flash to restore defaults.

---

### H. Controller/Teleop Not Working

**Symptom:** Robot doesn't move when using gamepad, even though ROS nodes seem okay.

1.  **Controller Not Connected/Detected:**
    *   **Cause:** Bluetooth pairing failed, USB cable loose, controller off, battery dead.
    *   **Solution:** Re-pair Bluetooth (use `bluetoothctl`: `scan on`, `pair`, `trust`, `connect`). Check USB cable. Charge/replace controller batteries. Verify detection with `ls /dev/input/js*`. Test with `jstest-gtk`.
2.  **Incorrect `joy_config`:**
    *   **Cause:** Launched `joy_teleop.launch.py` with the wrong config name (e.g., used `xbox` for a PS4 controller). Button mappings are incorrect.
    *   **Solution:** Stop the launch command (Ctrl+C). Find the correct config name (check `turtlebot4_teleop` package launch/config files or documentation). Relaunch with the correct `joy_config:=<name>`.
3.  **Teleop Node Not Running/Launched Correctly:**
    *   **Cause:** Forgot to launch the `joy_teleop.launch.py` file, launch command failed, ROS environment not sourced in the terminal where you launched it.
    *   **Solution:** Ensure the launch command is running in a terminal (either on workstation or SSH'd into robot, depending on method chosen). Check for errors in its output. Source ROS 2 (`source /opt/ros/jazzy/setup.bash`) before launching.
4.  **Topic Mismatch / Dead Man's Switch:**
    *   **Cause:** Teleop node isn't publishing to `/cmd_vel` (or the topic the Create 3 node expects), or you are not pressing the required dead man's switch button.
    *   **Solution:** Check teleop node's output/config for the published topic. Echo `/joy` topic (`ros2 topic echo /joy --once`) to see raw controller data. Echo `/cmd_vel` (`ros2 topic echo /cmd_vel --once`) to see if commands are being sent when buttons/sticks are pressed. Identify and hold the dead man's switch (often a shoulder button).
5.  **Create 3 Not Accepting Commands:**
    *   **Cause:** Create 3 base is in a fault state or requires a specific mode/enable command (usually handled by `create3_node` but could be an issue).
    *   **Solution:** Check `/diagnostics`. Power cycle Create 3 base.

---

### I. Specific Hardware Issues

1.  **LiDAR Not Spinning / No `/scan` Data:**
    *   **Cause:** Loose cable, motor failure, insufficient power, driver issue.
    *   **Solution:** Check LiDAR cable connection to top plate PCB and PCB to Pi. Check `/diagnostics` and node logs. Listen for motor sound. May require hardware replacement.
2.  **Camera Not Working / No Image Topics:**
    *   **Cause:** Loose USB cable, faulty camera, driver issue (`depthai`), power issue.
    *   **Solution:** Check/reseat USB cable firmly in Pi and camera. Check `lsusb` to see if camera is detected by OS. Check node logs (`camera_node` or similar) and `/diagnostics`. Try a different USB port on Pi. May require hardware replacement.
3.  **Create 3 Errors (Flashing Lights, Sounds):**
    *   **Cause:** See iRobot Create 3 documentation for error code meanings (lights/sounds). Could be battery issue, sensor obstruction (cliff/bump), internal fault.
    *   **Solution:** Consult Create 3 manual. Clean sensors. Check battery. Power cycle.
4.  **Display Screen Issues:**
    *   **Cause:** Loose ribbon cable, faulty screen, software agent controlling screen not running.
    *   **Solution:** Check ribbon cable connection (requires care, potentially removing casing). Check if related display software/service is running. Re-flash SD card. May require hardware replacement.

---

*If problems persist after trying relevant steps, consider seeking help on ROS Answers ([answers.ros.org](https://answers.ros.org/)) or the Clearpath Robotics community forums/support, providing detailed information about your setup, what you've tried, and specific error messages.*

## 12. Routine Operation and Charging

Once your TurtleBot 4 is set up and connected to your WiFi:

1.  **Powering On:**
    *   Ensure the battery is inserted.
    *   Press the power button on the Create 3 base.
    *   Wait for the robot to boot fully. The screen should display its IP address once connected to your WiFi.
    *   You can now SSH into the robot using its client IP address or interact with it via ROS 2 topics from your workstation.
2.  **Charging:**
    *   The TurtleBot 4 comes with an automatic docking station.
    *   **Manual Docking:** Gently push the robot onto the dock until the charging contacts engage. Observe the Create 3 charging indicators.
    *   **Automatic Docking (Using ROS 2):** The `irobot_create_msgs` package provides actions for autonomous docking. You can trigger docking via RViz (using a plugin if available), command-line tools, or your own code.
        *   Example Action Call (Requires `irobot_create_msgs` and action client setup): Send a goal to the `/dock` action server (check exact name with `ros2 action list`).
        *   Refer to Create 3 and TurtleBot 4 documentation for specific docking commands/actions.
    *   **Best Practice:** Keep the robot docked when not in use to ensure the battery remains charged. The Create 3 manages charging automatically.
3.  **Powering Off:**
    *   **Recommended:** SSH into the robot and run `sudo shutdown now` or `sudo poweroff`. Wait for the Raspberry Pi's activity lights to stop blinking before turning off the Create 3 base power button. This ensures a clean shutdown and reduces filesystem corruption risk.
    *   **Alternative:** Press and hold the Create 3 power button to turn off the base, which will also cut power to the Pi (less ideal than a clean shutdown).

## 13. Basic Maintenance

Regular maintenance ensures longevity and optimal performance:

1.  **Cleaning Sensors:**
    *   **LiDAR:** Gently wipe the rotating LiDAR cover with a microfiber cloth to remove dust. Ensure nothing obstructs its rotation. **Do not** spray liquids directly onto it.
    *   **Camera Lenses (OAK-D):** Wipe gently with a clean microfiber cloth. Avoid scratches.
    *   **Create 3 Sensors:** Regularly wipe the cliff sensors (underside), bump sensors (front), and optical floor tracking sensor (underside) with a dry or slightly damp cloth to remove dust and debris. Clogged cliff sensors can cause navigation errors or prevent undocking.
2.  **Wheels and Casters:**
    *   Check for hair, threads, or debris wrapped around the wheels and casters. Remove any obstructions carefully. Ensure they spin freely.
3.  **Charging Contacts:**
    *   Wipe the charging contacts on both the robot's underside and the docking station with a dry cloth periodically to ensure a good connection.
4.  **Battery:**
    *   Follow iRobot Create 3 guidelines for battery care. Avoid deep discharging frequently. Store partially charged if storing long-term.
5.  **Ventilation:**
    *   Ensure the Raspberry Pi's ventilation (if using a case with fan/heatsink) is not blocked. Overheating can cause performance issues.
6.  **Cable Checks:** Periodically re-check that the key cables (Pi-Create3 USB-C, Power, Sensor USBs) are secure, especially if the robot encounters significant bumps.

## 14. Understanding the ROS 2 System

To effectively use and debug your TurtleBot 4, understand its ROS 2 structure:

1.  **Key Nodes:** Run `ros2 node list` while the robot is running. Understand the purpose of major nodes:
    *   `/create3_node`: Interfaces with the iRobot Create 3 base (odometry, sensors, commands).
    *   `/lidar_node`: Interfaces with the RPLIDAR sensor.
    *   `/camera/camera_node` (or similar): Interfaces with the OAK-D camera (images, depth, potentially NN outputs).
    *   `/joy_linux_node`: Reads joystick input.
    *   `/teleop_twist_joy_node` (or similar, depends on launch): Converts joystick input to `/cmd_vel`.
    *   `/robot_state_publisher`: Publishes robot transforms (`tf`) based on joint states (URDF).
    *   `/turtlebot4_node`: Handles TurtleBot 4 specific functions (display, buttons, diagnostics aggregation).
    *   `/diagnostics_agg`: Aggregates system status messages.
2.  **Key Topics:** Use `ros2 topic list` and `ros2 topic echo <topic_name>`:
    *   `/odom`: Odometry data from the Create 3 (position estimate based on wheel movement).
    *   `/scan`: 2D LiDAR data.
    *   `/image_raw`, `/depth/image_raw`, `/color/image_raw`: Camera data streams (check exact names).
    *   `/diagnostics`: Health and status messages from various components.
    *   `/tf`, `/tf_static`: Coordinate frame transformations. Crucial for relating sensor data to the robot's position.
    *   `/cmd_vel`: The topic used to send velocity commands (linear x, angular z) to the robot base.
    *   `/joint_states`: Information about the robot's joints (wheel positions, etc.) used by `robot_state_publisher`.
3.  **Coordinate Frames (TF Tree):** Visualize the relationship between different parts of the robot and sensor data.
    *   Use RViz 2: Add `TF` display type.
    *   Use command line: `ros2 run tf2_tools view_frames` (generates a PDF diagram).
    *   Common frames: `odom` (odometry world frame), `base_link` (robot center), `laser_frame` (LiDAR), `camera_link` (camera base), etc. Understanding TF is vital for navigation and perception.
4.  **Launch Files:** Explore the launch files in `/opt/ros/jazzy/share/turtlebot4_bringup/launch/` (and related packages like `turtlebot4_navigation`, `turtlebot4_viz`). These orchestrate starting multiple nodes with specific configurations.

## 15. Software Updates

Keeping the software up-to-date is important for bug fixes, performance improvements, and new features.

1.  **Check for Updates:** Clearpath Robotics typically provides updated images or instructions for updating packages. Check their documentation periodically.
2.  **Update Ubuntu Packages (Standard System Update):**
    *   SSH into the robot.
    *   Run:
        ```bash
        sudo apt update
        sudo apt upgrade
        ```
    *   **Caution:** While generally safe, massive upgrades *could* occasionally introduce regressions or conflicts with ROS packages if dependencies change significantly. It's often safer to rely on full image updates provided by Clearpath unless you need specific security patches.
3.  **Update ROS 2 Packages:**
    *   If Clearpath provides updated ROS packages via their own Debian repositories, the `sudo apt upgrade` command might pull them in.
    *   If specific ROS packages need updating independently (less common for the base system), you might use `sudo apt install --only-upgrade <ros-package-name>`.
4.  **Firmware Updates (Create 3):**
    *   The iRobot Create 3 base may receive firmware updates. Follow iRobot's official procedures for this, which might involve connecting it directly to WiFi temporarily or using specific tools. Check the Create 3 documentation. These updates are separate from the Raspberry Pi OS/ROS updates.
5.  **Flashing a New Image:** For major version upgrades or to ensure a clean state, flashing a newer official TurtleBot 4 OS image (following the steps in Phase 1) is the most robust method.

## 16. Simple Programming & Next Steps

Now that your robot is running, you can start developing applications.

1.  **Workstation Setup:** Ensure your workstation has ROS 2 Jazzy installed and can communicate with the robot over the network (same WiFi, ROS_DOMAIN_ID matched or DDS configured).
2.  **Basic Publisher/Subscriber (Python/C++):**
    *   Write a simple ROS 2 node on your workstation that subscribes to `/scan` or `/odom` data from the robot.
    *   Write a simple node that publishes to `/cmd_vel` to make the robot move programmatically (start with very low speeds!).
    *   Follow the core ROS 2 Tutorials: [https://docs.ros.org/en/jazzy/Tutorials.html](https://docs.ros.org/en/jazzy/Tutorials.html)
3.  **Using RViz 2:**
    *   Launch RViz 2 on your workstation: `ros2 run rviz2 rviz2`
    *   Load the default TurtleBot 4 RViz configuration: `ros2 launch turtlebot4_viz rviz.launch.py` (check exact launch file name).
    *   Add displays for `/scan`, `/odom`, `/image_raw`, `/tf`, RobotModel, etc., to visualize the robot's state and sensor data.
4.  **SLAM (Mapping):**
    *   Run the SLAM launch file: `ros2 launch turtlebot4_navigation slam_sync.launch.py` (or `slam_async.launch.py`).
    *   Drive the robot around using the gamepad (launched separately).
    *   Visualize the map building process in RViz.
    *   Save the map using the map saver tool (often integrated or run separately: `ros2 run nav2_map_server map_saver_cli -f ~/my_map_name`).
5.  **Navigation (Autonomous Movement):**
    *   Run the Navigation launch file, providing your saved map: `ros2 launch turtlebot4_navigation nav2.launch.py map:=/path/to/your/my_map_name.yaml`
    *   Use the "2D Pose Estimate" tool in RViz to initialize the robot's position in the map.
    *   Use the "Nav2 Goal" tool in RViz to send navigation goals.
6.  **Explore TurtleBot 4 Packages:** Look at the code and launch files within the installed TurtleBot 4 packages (`/opt/ros/jazzy/share/`) to understand how different functionalities are implemented.
