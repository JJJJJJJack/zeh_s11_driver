#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <std_msgs/Float32MultiArray.h>

//#define DEBUG_OUTPUT

#define SENSOR_NUMBER 4
#define PACKET_LENGTH (6+SENSOR_NUMBER*2) // 数据帧长度
#define START_BYTE 0xFF
#define CMD_BYTE 0x86

class SensorDriver {
private:
    serial::Serial ser;
    ros::NodeHandle nh;
    ros::Publisher temp_pub;
    ros::Publisher humidity_pub;
    ros::Publisher gas_pub;
    std::string port;
    int baudrate;

    // 校验和计算（与文档一致）
    unsigned char checksum(unsigned char *data, unsigned char len) {
        unsigned short sum = 0;
        for (int i = 1; i < len; i++) {
            sum += data[i];
        }
        return (~(sum % 256) + 1);
    }

public:
    SensorDriver() : port("/dev/ttyUSB0"), baudrate(9600) {
        temp_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 10);
        humidity_pub = nh.advertise<sensor_msgs::RelativeHumidity>("humidity", 10);
        gas_pub = nh.advertise<std_msgs::Float32MultiArray>("gas_concentrations", 10);
        
        // 从参数服务器读取串口配置
        nh.param<std::string>("port", port, "/dev/ttyUSB0");
        nh.param<int>("baudrate", baudrate, 9600);

        try {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (const std::exception &e) {
            ROS_FATAL("Failed to open serial port: %s", e.what());
            ros::shutdown();
        }
    }

    void run() {
        uint8_t buffer[PACKET_LENGTH+1];
        while (ros::ok()) {
            if (ser.available()) {
                // 寻找起始位0xFF
                ser.read(buffer, 1);
                if (buffer[0] != START_BYTE) continue;

                // 读取剩余字节
                size_t bytes_read = ser.read(buffer + 1, PACKET_LENGTH);
                if (bytes_read != PACKET_LENGTH) {
                    ROS_WARN("Incomplete packet");
                    continue;
                }
#ifdef DEBUG_OUTPUT
                printf("%d\n", PACKET_LENGTH);
                for(int i=0; i < PACKET_LENGTH+1; i++){
                    printf("%x ", buffer[i]);
                }
                printf("\n");
                printf("%x   %x\n", buffer[PACKET_LENGTH], checksum(buffer, PACKET_LENGTH) );
#endif
                // 校验命令字和校验和
                if (buffer[1] != CMD_BYTE || buffer[PACKET_LENGTH] != checksum(buffer, PACKET_LENGTH)) {
                    ROS_WARN("Invalid packet");
                    continue;
                }

                // 解析数据
                std_msgs::Float32MultiArray gas_msg;
                gas_msg.data.resize(SENSOR_NUMBER); // S1-S8（或替代位置）

                // 解析气体浓度（示例仅解析S1-S4，其余类似）
                for (int i = 0; i < SENSOR_NUMBER; i++) {
                    int high = buffer[2 + 2 * i];
                    int low = buffer[3 + 2 * i];
                    // 检查故障码0xFF00
                    if (high == 0xFF && low == 0x00) {
                        gas_msg.data[i] = NAN;
                    } else {
                        // 假设分辨率为1（需根据实际模组调整）
                        gas_msg.data[i] = (high * 256.0 + low) * 1.0; 
                    }
                }

                // 解析温度（单位：℃）
                int temp_high = buffer[2+SENSOR_NUMBER*2];
                int temp_low = buffer[3+SENSOR_NUMBER*2];
                float temperature = (temp_high * 256.0 + temp_low - 500) * 0.1;

                // 解析湿度（单位：%RH）
                int humid_high = buffer[4+SENSOR_NUMBER*2];
                int humid_low = buffer[5+SENSOR_NUMBER*2];
                float humidity = (humid_high * 256.0 + humid_low) * 0.1;

                // 发布消息
                sensor_msgs::Temperature temp_msg;
                temp_msg.temperature = temperature;
                temp_msg.header.stamp = ros::Time::now();
                temp_pub.publish(temp_msg);

                sensor_msgs::RelativeHumidity humid_msg;
                humid_msg.relative_humidity = humidity / 100.0; // 转换为小数形式
                humid_msg.header.stamp = ros::Time::now();
                humidity_pub.publish(humid_msg);

                gas_pub.publish(gas_msg);
            }
            ros::spinOnce();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "zeh_s11_driver");
    SensorDriver driver;
    driver.run();
    return 0;
}