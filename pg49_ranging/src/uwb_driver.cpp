#include <ros/ros.h>
#include <serial/serial.h>
#include <pg49_ranging/DistanceStamped.h>
#include <vector>
#include <cstdint>
#include <string>

namespace {

uint16_t read_uint16_be(const std::vector<uint8_t>& data, size_t offset) {
    return (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
}

bool has_valid_crc(const std::vector<uint8_t>& frame) {
    const size_t crc_offset = frame.size() - 2;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < crc_offset; ++i) {
        crc ^= frame[i];
        for (int bit = 0; bit < 8; ++bit) {
            const bool low_bit = (crc & 1) != 0;
            crc >>= 1;
            if (low_bit) {
                crc ^= 0xA001;
            }
        }
    }
    const uint16_t received = frame[crc_offset] |
        (static_cast<uint16_t>(frame[crc_offset + 1]) << 8);
    return crc == received;
}

}  // namespace

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
        pub_distance_stamped_ =
            nh_.advertise<pg49_ranging::DistanceStamped>("/uwb/distance_stamped", 10);
    }

    bool parse_frame(const std::vector<uint8_t>& data, const ros::Time& received_at) {
        try {
            const uint16_t output_flags = read_uint16_be(data, 5);
            const uint16_t ranging_status = read_uint16_be(data, 11);
            const bool has_position = (output_flags & 0x01) != 0;
            const bool has_ranges = (output_flags & 0x02) != 0;
            const bool anchor_a_valid = (ranging_status & 0x01) != 0;
            const size_t range_offset = has_position ? 19 : 13;
            if (!has_ranges || !anchor_a_valid || range_offset + 2 > data.size() - 2) {
                return false;
            }

            // 1. 物理层解析
            uint16_t raw_dist_cm = read_uint16_be(data, range_offset);
            double raw_dist = raw_dist_cm / 100.0;

            if (raw_dist < 0.1) {
                return false;
            }

            // 2. 零偏补偿层
            double corrected_dist = raw_dist + bias_offset_;

            // 3. 滤波算法层
            double filtered_dist = use_kalman_ ? kf_.update(corrected_dist) : corrected_dist;

            // 4. ROS 话题发布
            pg49_ranging::DistanceStamped stamped_msg;
            stamped_msg.header.stamp = received_at;
            stamped_msg.distance = static_cast<float>(filtered_dist);
            pub_distance_stamped_.publish(stamped_msg);

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
                const ros::Time received_at = ros::Time::now();
                buffer.insert(buffer.end(), temp_buf.begin(), temp_buf.end());

                while (buffer.size() >= 5) {
                    const size_t byte_count = buffer[2];
                    if (buffer[0] != 0x01 || buffer[1] != 0x03 ||
                        buffer[3] != 0xCA || buffer[4] != 0xDA ||
                        byte_count < 10 || byte_count % 2 != 0) {
                        buffer.erase(buffer.begin());
                        continue;
                    }

                    // Three header bytes, the register data, and two CRC bytes.
                    const size_t frame_len = 3 + byte_count + 2;
                    if (buffer.size() < frame_len) {
                        break;
                    }

                    const std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_len);
                    if (!has_valid_crc(frame)) {
                        buffer.erase(buffer.begin());
                        continue;
                    }
                    parse_frame(frame, received_at);
                    buffer.erase(buffer.begin(), buffer.begin() + frame_len);
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_distance_stamped_;

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
