reference：

- https://github.com/Ericsii/ros_wit_imu_node
- https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2





### ✅ 第一步：硬件连接（TTL方式）

根据你上传的 DatasheetWT901C TTL Datasheet：

- VCC → 5V
- GND → GND
- RX → 接 USB-TTL 模块的 TX
- TX → 接 USB-TTL 模块的 RX

确保 USB-TTL 模块已插入 Ubuntu，并能识别 `/dev/ttyUSB0` 或 `/dev/ttyACM0` 等设备路径。

```bash
ls /dev/ttyUSB*
```

## Windows版官方插件检查连通性

- [WitMotion New Software.zip](https://drive.google.com/drive/folders/1TLutidDBd_tDg5aTXgjvkz63OVt5_8ZZ)
- [CH340& CP2102 Driver.zip](https://drive.google.com/file/d/1JidopB42R9EsCzMAYC3Ya9eJ8JbHapRF/view?pli=1)

下载完软件，插上USB后，可以查看电脑中`device manager` 中的 `COM and LPT`， 查看是哪个穿行接口

打开WitMotion软件之后，选择对应的型号，这里是`WT901-TTL`，然后点击 Search devices，最后会跳出一个COMX，选择的是9600 的波特率，最后点击左侧的加号，而不是右侧的加号，注意！！！
![MygOKr4By5](https://github.com/user-attachments/assets/b0b84958-3d12-43fc-8cbf-b05fabc2154f)


![MygOKr4By5](./resources/images/MygOKr4By5.png)

最后就会有数据显示了

reference: https://wit-motion.yuque.com/wumwnr/docs/qynnu9?#%20%E3%80%8A%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E3%80%8B



## ROS 2 Humble 安装Witmotion IMU

参考：

- Ros1: https://github.com/ElettraSciComp/witmotion_IMU_ros?tab=readme-ov-file
- Ros2: https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2



按照ros2的安装步骤来：

```bash
cd ~/ros2_ws/src
git clone -b ros2 --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git witmotion_ros
colcon build --packages-select witmotion_ros
source install/setup.bash
```

If compilation fails, first check the directory `src/witmotion_ros/witmotion-uart-qt`. If it is empty, the recursive clone failed, and you should manually clone the underlying library from the repository https://github.com/ElettraSciComp/witmotion_IMU_QT into this directory. **IMPORTANT!** Please beware of the directory name, the `CMakeLists` file refers exactly to the name `witmotion-uart-qt` specified in the target import section.



官方的启动代码是：`ros2 launch witmotion_ros wt61c.py`

这里我是：

```bash
ros2 run witmotion_ros witmotion_ros_node --ros-args --params-file ~/ros2_ws/src/witmotion_ros/config/wt901.yml
```



```bash
RCL_LOG_LEVEL=debug ros2 run witmotion_ros witmotion_ros_node --ros-args --params-file ~/ros2_ws/src/witmotion_ros/config/wt901.yml
```







> 注意！！！
>
> 参考这个链接，将官方的文档中这个参数`use_native_orientation: True # 311 edit: default -> true (ref: https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/34)`改为 `false`，`ros2 topic echo /imu` 才会出数据，不然一直不输出，为空
>
> 参考：https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/42
>
> - `use_native_orientation` - instructs the node to use the native quaternion orientation measurement from the sensor instead of synthesized from Euler angles. **NOTE**: if this setting is enabled bu the sensor does not produce orientation in the quaternion format, the IMU message will never be published!
>
>   use_native_orientation - 指示节点使用来自传感器的本地四元数方向测量，而不是根据欧拉角合成。注意：如果启用此设置，但传感器不以四元数格式生成方向，则 IMU 信息将永远不会发布



https://wiki.ros.org/witmotion_ros/Troubleshooting



wt901.yml:

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

