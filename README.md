# TI mmWave ROS Package (Customized)

#### Auhor and Maintainer: Leo Zhang
#### Organization: University of Arizona
#### Email: dr.leo.zhang@outlook.com
---
### Most recent change:

Add support for XWR18XX devices. SDK version: 3.2.0.4.

---
Initially derived from TI's origin ROS package in Industrial Toolbox 2.3.0 (new version available [Industrial Toolbox 2.5.2](http://dev.ti.com/tirex/#/?link=Software%2FmmWave%20Sensors%2FIndustrial%20Toolbox)).

### Differences from origin TI's version:
1. Added all radar parameters from calculations and can be read from `rosparam get`.
2. Added Doppler data from detecting targets and form a customized ROS message `/ti_mmwave/radar_scan`.
3. Added support for multiple radars working together.
4. Added support for camera overlay (for sensor fusion).
5. Working with xWR1443 and xWR1642 ES1.0 and ES2.0 (ES1.0 is deprecated from TI)
---
### Available devices:
```
TI mmWave AWR1443BOOST
TI mmWave AWR1642BOOST
TI mmWave AWR1642BOOST ES2.0/3.0 EVM (not tested)
TI mmWave AWR1642BOOST ES2.0 EVM
TI mmWave AWR1843BOOST ES1.0 EVM
```
---
### Quick start guide (AWR1642BOOST ES2.0 EVM):
1. Mount AWR1642BOOST ES2.0 EVM (as below), connect 5V/2.5A power supply and connect a micro-USB cable to host Ubuntu 16.04 LTS with [ROS Kinetic](http://wiki.ros.org/kinetic).
   
![](https://github.com/radar-lab/ti_mmwave_rospkg/raw/master/auxiliary/mounting.jpg "AWR1642 Mounting")

Note: Tested with Ubuntu 16.04 LTS with ROS Kinectic and Ubuntu 18.04 LTS with [ROS Melodic](http://wiki.ros.org/melodic)

2. Download SDK 2.0 or above (suggested SDK 2.1) from [here](http://www.ti.com/tool/MMWAVE-SDK) and use [UNIFLASH](http://www.ti.com/tool/UNIFLASH) to flash xwr16xx_mmw_demo.bin to your device. **Do not forget SOP2 jumper when flashing.**

Note:
AWR1642 ES1.0 (usually purchased before May 2018) uses SDK 1.2. AWR1642 ES2.0 (usually purchased after May 2018) uses SDK 2.0. Same applies to AWR1443. (You can refer to [this thread](https://e2e.ti.com/support/sensors/f/1023/t/692195?tisearch=e2e-sitesearch&keymatch=%20user:356347))

3. Clone this repo and ROS serial onto your `<workspace dir>/src`:

```
git clone https://github.com/radar-lab/ti_mmwave_rospkg.git
git clone https://github.com/wjwwood/serial.git
```
4. Go back to `<workspace dir>`:

```
catkin_make && source devel/setup.bash
echo "source <workspace_dir>/devel/setup.bash" >> ~/.bashrc
```

5. Enable command and data ports on Linux:
```
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```
Note: If multiple sensors are used, enable additional ports `/dev/ttyACM2` and `/dev/ttyACM3`, etc. the same as this step.

6. Launch AWR1642 short range config:
```
roslaunch ti_mmwave_rospkg 1642es2_short_range.launch
```

Note: If you want to build your own config, use [mmWave Demo Visualizer](https://dev.ti.com/mmwavedemovisualizer) and link the launch file to the config.

7. ROS topics can be accessed as follows:
```
rostopic list
rostopic echo /ti_mmwave/radar_scan
```
8. ROS parameters can be accessed as follows:
```
rosparam list
rosparam get /ti_mmwave/max_doppler_vel
```

Note: AWR1843 requires SDK 3.2.0.4, which has different output format. The later release will improve this part.

---
### Message format:
```
header: 
  seq: 6264
  stamp: 
    secs: 1538888235
    nsecs: 712113897
  frame_id: "ti_mmwave"   # Frame ID used for multi-sensor scenarios
point_id: 17              # Point ID of the detecting frame (Every frame starts with 0)
x: 8.650390625            # Point x coordinates in m (front from antenna)
y: 6.92578125             # Point y coordinates in m (left/right from antenna, right positive)
z: 0.0                    # Point z coordinates in m (up/down from antenna, up positive)
range: 11.067276001       # Radar measured range in m
velocity: 0.0             # Radar measured range rate in m/s
doppler_bin: 8            # Doppler bin location of the point (total bins = num of chirps)
bearing: 38.6818885803    # Radar measured angle in degrees (right positive)
intensity: 13.6172780991  # Radar measured intensity in dB
```
---
### Troubleshooting
1.
```
mmWaveCommSrv: Failed to open User serial port with error: IO Exception (13): Permission denied
mmWaveCommSrv: Waiting 20 seconds before trying again...
```
This happens when serial port is called without superuser permission, do the following steps:
```
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```
2.
```
mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')
mmWaveQuickConfig: Response: 'sensorStop
'?`????`????`???~' is not recognized as a CLI command
mmwDemo:/>'
```
When this happens, re-run the command you send to sensor. If it continues, shut down and restart the sensor.

---
### Multiple devices support (dual AWR1642 ES2.0 EVM):
1. Connect two devices and try `ll /dev/serial/by-id` or `ls /dev`. In this case, `/dev/ttyACM0` to `/dev/ttyACM3` should shown.
2. To avoid serial port confliction, you need to launch devices separately. So for first device (it will open rviz):

```
roslaunch ti_mmwave_rospkg multi_1642_0.launch 
```
3. Change radars' location in first three arguments `<node pkg="tf" type="static_transform_publisher" name="radar_baselink_0" args="0 -1 0 0 0 0 ti_mmwave_pcl ti_mmwave_0 100"/>` (stands for x,y,z for positions) in launch file `multi_1642_1.launch`. And launch second device:

```
roslaunch ti_mmwave_rospkg multi_1642_1.launch 
```

Note: As serial connection and the original code, you need to launch devices separately using different launch files.

---
### Camera overlay support (working with USB camera or CV camera):
1. Download and build USB camera repo [here](https://github.com/radar-lab/usb_webcam`). And set parameters of camera in `<usb_webcam dir>/launch/usb_webcam.launch`.
2. To test the device image working, try:
```
roslaunch usb_webcam usb_webcam.launch
rosrun rqt_image_view rqt_image_view  
```
3. Make sure you have done [ROS camera calibration](http://wiki.ros.org/camera_calibration) and create a `*.yaml` configuration file accordingly.
4. Launch radar-camera system using:
```
roslaunch ti_mmwave_rospkg camera_overlay.launch
```

---
### Changelog:

```
v3.3.0

Add support for XWR18XX devices. SDK version: 3.2.0.4.

v3.2.2
Fix bugs and update README.

v3.2.1
Support camera overlay over 3D 1443s.

v3.2.0
Added camera overlay support.

v3.1.0
Strengthened code.

v3.0.0
Added README.
Improved rviz looking for point cloud data.
Added support for multiple radars working together. 
Improved radar's all around working conditions.

v2.0.0
Added support for ES2.0 EVM devices.

v1.0.0
Added Doppler from TI's mmWave radars.
```