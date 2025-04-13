# ZEHS11 Multi-Sensor ROS Driver

ROS driver for **Winsen ZEHS11 Multi-Sensor Converter Board**, supporting UART communication to publish temperature, humidity, and gas concentrations.

---

## 传感器概述
- **型号**: ZEHS11  
- **功能**: 支持8路气体模组（如ZE03系列、MH-Z19等）及温湿度数据的采集与转换。  
- **通信协议**: 默认主动上传模式，UART/485输出。  

---

## 安装与依赖

### 安装依赖库（需替换为您的ROS版本，如noetic、melodic等）
```sudo apt-get install ros-<your_ros_distro>-serial```

## 使用方法
### 启动驱动节点
```roslaunch zeh_s11_driver zeh_s11.launch```
动态配置参数（可选）
指定串口设备：```port:=/dev/ttyUSB1```
指定波特率：```baudrate:=115200```

### 运行示例
```roslaunch zeh_s11_driver zeh_s11.launch port:=/dev/ttyACM0 baudrate:=9600```

## 发布的话题说明

本ROS驱动节点发布以下话题：

| 话题名称               | 消息类型                     | 说明                                                                 |
|------------------------|------------------------------|----------------------------------------------------------------------|
| `/temperature`         | `sensor_msgs/Temperature`    | 温度数据，单位为摄氏度（℃）                                         |
| `/humidity`            | `sensor_msgs/RelativeHumidity` | 湿度数据，范围为 `0.0~1.0`（对应百分比湿度，例如 `0.5` 表示 50%RH）  |
| `/gas_concentrations`  | `std_msgs/Float32MultiArray` | 气体浓度数组，数组索引 `0~7` 对应传感器位置 `S1~S8`，单位默认 `ppm`  |
