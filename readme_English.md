Reference：

- https://github.com/Ericsii/ros_wit_imu_node
- https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2



### Step 1  The hardware connection (TTL method)

According to the DatasheetWT901C TTL Datasheet you uploaded:

- VCC → 5V
- GND → GND
- RX → TX of USB-TTL module
- TX → RX of the USB-TTL module.

Make sure the USB-TTL module is plugged into Ubuntu and recognizes device paths such as `/dev/ttyUSB0` or `/dev/ttyACM0`.

```bash
ls /dev/ttyUSB*
```



### Step 2 The official plugin for Windows checks connectivity

- [WitMotion New Software.zip](https://drive.google.com/drive/folders/1TLutidDBd_tDg5aTXgjvkz63OVt5_8ZZ)
- [CH340& CP2102 Driver.zip](https://drive.google.com/file/d/1JidopB42R9EsCzMAYC3Ya9eJ8JbHapRF/view?pli=1)

After downloading the software and plugging in the USB, you can check the `COM and LPT` in the `device manager` of your computer, to see which interface you are traveling through.

After opening the WitMotion software, select the corresponding model, here is `WT901-TTL`, then click Search devices, finally a COMX will pop up, select the baud rate of 9600, and finally click on the left side of the red box I marked with a plus sign, instead of the right side of the plus sign, pay attention!

![MygOKr4By5](./resources/images/MygOKr4By5.png)

Finally, there will be data to show

reference: https://wit-motion.yuque.com/wumwnr/docs/qynnu9?#%20%E3%80%8A%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E3%80%8B



### Step 3 ROS 2 Humble Install Witmotion IMU

Reference

- Ros1: https://github.com/ElettraSciComp/witmotion_IMU_ros?tab=readme-ov-file
- Ros2: https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2



Follow the installation steps for ros2, only the `witmotion_ros` package will be compiled here:

```bash
cd ~/ros2_ws/src
git clone -b ros2 --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git witmotion_ros
colcon build --packages-select witmotion_ros
source install/setup.bash
```

If compilation fails, first check the directory `src/witmotion_ros/witmotion-uart-qt`. If it is empty, the recursive clone failed, and you should manually clone the underlying library from the repository https://github.com/ElettraSciComp/witmotion_IMU_QT into this directory. **IMPORTANT!** Please beware of the directory name, the `CMakeLists` file refers exactly to the name `witmotion-uart-qt` specified in the target import section.



The official startup code is: `ros2 launch witmotion_ros wt61c.py`

Here I am: (since I have the wt901c version of imu, I need to use this parameter configuration file in the launch node)

```bash
ros2 run witmotion_ros witmotion_ros_node --ros-args --params-file ~/ros2_ws/src/witmotion_ros/config/wt901.yml
```

Or use the debug method if you encounter problems

```bash
RCL_LOG_LEVEL=debug ros2 run witmotion_ros witmotion_ros_node --ros-args --params-file ~/ros2_ws/src/witmotion_ros/config/wt901.yml
```



> Attention!!!
>
> Refer to this link, change this parameter `use_native_orientation: True # 311 edit: default -> true (ref: https://github.com/ElettraSciComp/witmotion_IMU_ros/ issues/34)` to `false`, `ros2 topic echo /imu` only then the data will come out, otherwise it never outputs, it's empty, I was stuck here for two hours, haha, then realized it's a simple problem.
>
> Reference: https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/42
>
> - `use_native_orientation` - instructs the node to use the native quaternion orientation measurement from the sensor instead of synthesized from Euler angles. **NOTE**: if this setting is enabled bu the sensor does not produce orientation in the quaternion format, the IMU message will never be published!
>
>   use_native_orientation - Instructs the node to use local quaternion direction measurements from the sensor, rather than synthesizing them based on Euler angles. Note: If this setting is enabled but the sensor does not generate directions in quaternion format, IMU information will never be published!



https://wiki.ros.org/witmotion_ros/Troubleshooting



wt901.yml:

- **port**: ttyUSB0
- **baud_rate**: 9600 # baud

```yml
witmotion:
  ros__parameters:
    port: ttyUSB0
    baud_rate: 9600 # baud
    polling_interval: 50 # ms
    timeout_ms: 150 # ms
    restart_service_name: /restart_imu
    imu_publisher:
      topic_name: /imu
      frame_id: imu
      use_native_orientation: false
      measurements:
        acceleration:
          enabled: true
          covariance: [0.0364, 0.0, 0.0, 0.0, 0.0048, 0.0, 0.0, 0.0, 0.0796]
        angular_velocity:
          enabled: true
          covariance: [0.0663, 0.0, 0.0, 0.0, 0.1453, 0.0, 0.0, 0.0, 0.0378]
        orientation:
          enabled: true
          covariance: [0.0479, 0.0, 0.0, 0.0, 0.0207, 0.0, 0.0, 0.0, 0.0041]
    temperature_publisher:
      enabled: true
      topic_name: /temperature
      frame_id: base_link
      from_message: magnetometer # acceleration, angular_vel, orientation, magnetometer
      variance: 0.01829
      coefficient: 1.0 # Linear calibration parameters: coefficient
      addition: 0.0 # and addendum
    magnetometer_publisher:
      enabled: true
      topic_name: /magnetometer
      frame_id: compass
      coefficient: 0.00000001 # Linear calibration parameters: coefficient
      addition: 0.0 # and addendum
      covariance:
        [0.000000187123, 0.0, 0.0, 0.0, 0.000000105373, 0.0, 0.0, 0.0, 0.000000165816]
    barometer_publisher:
      enabled: false
      topic_name: /barometer
      frame_id: base_link
      variance: 0.001
      coefficient: 1.0 # Linear calibration parameters: coefficient
      addition: 0.0 # and addendum
    altimeter_publisher:
      enabled: false
      topic_name: /altitude
      coefficient: 1.0 # Linear calibration parameters: coefficient
      addition: 0.0 # and addendum
    orientation_publisher:
      enabled: true
      topic_name: /orientation
    gps_publisher:
      enabled: false
      navsat_fix_frame_id: world
      navsat_fix_topic_name: /gps
      navsat_altitude_topic_name: /gps_altitude
      navsat_satellites_topic_name: /gps_satellites
      navsat_variance_topic_name: /gps_variance
      ground_speed_topic_name: /gps_ground_speed
    rtc_publisher:
      enabled: false
      topic_name: /witmotion_clock
```

