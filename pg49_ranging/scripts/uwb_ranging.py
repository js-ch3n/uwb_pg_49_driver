#!/usr/bin/env python3
import rospy
import serial
import struct
from pg49_ranging.msg import DistanceStamped

# ====================== 卡尔曼滤波器 ======================
class KalmanFilter:
    def __init__(self, process_variance=0.005, measurement_variance=0.04, initial_estimate=0.0):
        self.process_variance = process_variance          # Q
        self.measurement_variance = measurement_variance  # R
        self.estimate = initial_estimate
        self.error = 1.0

    def update(self, measurement):
        self.error += self.process_variance
        kalman_gain = self.error / (self.error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.error = (1 - kalman_gain) * self.error
        return self.estimate


class UWBParser:
    def __init__(self):
        rospy.init_node('pg49_uwb_ranging', anonymous=True)
        
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        
        # ================== 参数配置区域 ==================
        self.use_kalman = True                     # 开启/关闭卡尔曼滤波 (True: 开启 | False: 关闭)
        self.process_variance = 0.005              # Q: 过程噪声 (调小更平滑，调大响应快)
        self.measurement_variance = 0.04           # R: 测量噪声 (结合 5-8cm 残差波动推荐 0.04)
        
        # UWB 零偏补偿值 (单位: 米)
        # 之前的系统固定零偏为 -0.276m，因此这里配置 +0.276 进行抵消
        self.bias_offset = 0.276                  
        # =================================================
        
        # 初始化卡尔曼滤波器，传入上面配置的 Q 和 R
        self.kf = KalmanFilter(
            process_variance=self.process_variance,
            measurement_variance=self.measurement_variance
        )
        
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
        self.ser.flushInput()
        
        self.pub_distance = rospy.Publisher('/uwb/distance_stamped', DistanceStamped, queue_size=10)
        
        rospy.loginfo(f"PG4.9 UWB 测距节点已启动")
        rospy.loginfo(f" -> 零偏补偿值: {self.bias_offset:+.3f} m")
        rospy.loginfo(f" -> 卡尔曼滤波: {'开启' if self.use_kalman else '关闭'} | Q={self.process_variance}, R={self.measurement_variance}")

    def parse_frame(self, data, received_at):
        if len(data) < 20 or data[0] != 0x01 or data[1] != 0x03:
            return False

        try:
            if data[3:5] != b'\xca\xda':
                return False

            # 跳过 tag_id 和位置信息（直接定位到距离部分）
            pos_idx = 13
            flag = struct.unpack('>H', data[5:7])[0]
            
            if flag & 0x01 and len(data) > pos_idx + 6:
                pos_idx += 6  # 跳过位置数据

            # 只取第一个距离
            if pos_idx + 2 > len(data) - 2:
                return False
                
            # 1. 物理层解析（得到米制原始距离）
            raw_dist = struct.unpack('>H', data[pos_idx:pos_idx+2])[0] / 100.0

            if raw_dist < 0.1:
                return False

            # 2. 零偏补偿层
            corrected_dist = raw_dist + self.bias_offset

            # 3. 滤波算法层
            if self.use_kalman:
                filtered_dist = self.kf.update(corrected_dist)
            else:
                filtered_dist = corrected_dist

            # 4. ROS 话题发布
            dist_msg = DistanceStamped()
            dist_msg.header.stamp = received_at
            dist_msg.distance = filtered_dist
            self.pub_distance.publish(dist_msg)

            rospy.loginfo_throttle(0.8, f"UWB 测距(已校准): {filtered_dist:.3f} m")

            return True

        except Exception:
            return False

    def run(self):
        buffer = b''
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                received_at = rospy.Time.now()
                buffer += chunk

                while len(buffer) > 30:
                    if buffer[:2] == b'\x01\x03':
                        byte_count = buffer[2]
                        frame_len = 3 + byte_count + 2
                        if len(buffer) >= frame_len:
                            self.parse_frame(buffer[:frame_len], received_at)
                            buffer = buffer[frame_len:]
                        else:
                            break
                    else:
                        buffer = buffer[1:]
            rate.sleep()


if __name__ == '__main__':
    try:
        node = UWBParser()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点异常: {e}")
