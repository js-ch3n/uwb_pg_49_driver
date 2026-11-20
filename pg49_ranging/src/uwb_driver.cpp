#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <cstdint>
#include <string>

// ====================== 卡尔曼滤波器 ======================
class KalmanFilter {
public:
    KalmanFilter() : q_(0.005), r_(0.04), estimate_(0.0), error_(1.0) {}
    KalmanFilter(double process_variance, double measurement_variance, double initial_estimate = 0.0)
        : q_(process_variance), r_(measurement_variance), estimate_(initial_estimate), error_(1.0) {}

    double update(double measurement) {
        error_ += q_;
        double kalman_gain = error_ / (error_ + r_);
        estimate_ = estimate_ + kalman_gain * (measurement - estimate_);
        error_ = (1.0 - kalman_gain) * error_;
        return estimate_;
    }

private:
    double q_;          // 过程噪声 Q
    double r_;          // 测量噪声 R
    double estimate_;   // 估计值
    double error_;      // 估计误差
};

// ====================== UWB 解析节点 ======================
class UWBParser {
public:
    UWBParser() : nh_("~") {
        // ================== 参数读取区域 ==================
        // 使用 nh_.param(参数名, 变量名, 默认值) 读取私有参数
        nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_.param<int>("baudrate", baudrate_, 115200);
        nh_.param<bool>("use_kalman", use_kalman_, true);
        nh_.param<double>("process_variance", process_variance_, 0.005);
        nh_.param<double>("measurement_variance", measurement_variance_, 0.04);
        nh_.param<double>("bias_offset", bias_offset_, 0.276);
        // =================================================

        // 初始化卡尔曼滤波器
        kf_ = KalmanFilter(process_variance_, measurement_variance_);

        // 初始化串口
        try {
            ser_.setPort(port_);
            ser_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(50); // timeout = 0.05s
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Failed to open port: " << port_ << ", Reason: " << e.what());
            ros::shutdown();
            return;
        }

        if (ser_.isOpen()) {
            ser_.flushInput();
            ROS_INFO_STREAM("PG4.9 UWB ranging node started");
            ROS_INFO(" -> Port: %s | Baudrate: %d", port_.c_str(), baudrate_);
            ROS_INFO(" -> Bias offset: %+0.3f m", bias_offset_);
            ROS_INFO(" -> Kalman filter: %s | Q=%f, R=%f", use_kalman_ ? "ON" : "OFF", process_variance_, measurement_variance_);
        }

        // 距离发布者
        pub_distance_ = nh_.advertise<std_msgs::Float32>("/uwb/distance", 10);
    }

    bool parse_frame(const std::vector<uint8_t>& data) {
        if (data.size() < 20 || data[0] != 0x01 || data[1] != 0x03) {
            return false;
        }

        try {
            if (data[3] != 0xCA || data[4] != 0xDA) {
                return false;
            }

            uint16_t flag = (static_cast<uint16_t>(data[5]) << 8) | data[6];
            size_t pos_idx = 13;

            if ((flag & 0x01) && (data.size() > pos_idx + 6)) {
                pos_idx += 6; 
            }

            if (pos_idx + 2 > data.size() - 2) {
                return false;
            }

            // 1. 物理层解析
            uint16_t raw_dist_cm = (static_cast<uint16_t>(data[pos_idx]) << 8) | data[pos_idx + 1];
            double raw_dist = raw_dist_cm / 100.0;

            if (raw_dist < 0.1) {
                return false;
            }

            // 2. 零偏补偿层
            double corrected_dist = raw_dist + bias_offset_;

            // 3. 滤波算法层
            double filtered_dist = use_kalman_ ? kf_.update(corrected_dist) : corrected_dist;

            // 4. ROS 话题发布
            std_msgs::Float32 dist_msg;
            dist_msg.data = static_cast<float>(filtered_dist);
            pub_distance_.publish(dist_msg);

            ROS_INFO_THROTTLE(0.8, "UWB Distance (Calibrated): %.3f m", filtered_dist);

            return true;
        } catch (...) {
            return false;
        }
    }

    void run() {
        std::vector<uint8_t> buffer;
        ros::Rate rate(100);

        while (ros::ok()) {
            if (!ser_.isOpen()) {
                ROS_WARN_THROTTLE(1.0, "Serial port is not open, skipping loop");
                rate.sleep();
                continue;
            }

            size_t available_bytes = ser_.available();
            if (available_bytes > 0) {
                std::vector<uint8_t> temp_buf;
                ser_.read(temp_buf, available_bytes);
                buffer.insert(buffer.end(), temp_buf.begin(), temp_buf.end());

                while (buffer.size() > 30) {
                    if (buffer[0] == 0x01 && buffer[1] == 0x03) {
                        uint8_t byte_count = buffer[2];
                        size_t frame_len = 5 + byte_count + 2;

                        if (buffer.size() >= frame_len) {
                            std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_len);
                            parse_frame(frame);
                            buffer.erase(buffer.begin(), buffer.begin() + frame_len);
                        } else {
                            break; 
                        }
                    } else {
                        buffer.erase(buffer.begin());
                    }
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_distance_;

    serial::Serial ser_;
    std::string port_;
    int baudrate_;

    bool use_kalman_;
    double process_variance_;
    double measurement_variance_;
    double bias_offset_;

    KalmanFilter kf_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pg49_uwb_ranging");
    try {
        UWBParser parser;
        parser.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Node exception: %s", e.what());
    }
    return 0;
}