#!/usr/bin/env python3
import rospy
import serial
import struct
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped


class KalmanFilter:
    def __init__(self, process_variance=0.015, measurement_variance=0.08):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
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
        
        # ================== Configuration ==================
        self.use_kalman = True                    # Enable Kalman Filter
        self.record_count = 1000                  # Number of points to collect before plotting
        # ===================================================
        
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
        self.ser.flushInput()
        
        self.pub_distances = rospy.Publisher('/uwb/distances', Float32MultiArray, queue_size=10)
        
        self.last_distances = None
        self.last_tag_id = None
        
        # Data recording for plotting
        self.raw_data = []
        self.filtered_data = []
        self.kalman_filters = {}
        
        rospy.loginfo(f"PG4.9 UWB Node Started | Kalman Filter: {'Enabled' if self.use_kalman else 'Disabled'} | Collecting {self.record_count} points for plot")

    def get_kalman(self, idx):
        if idx not in self.kalman_filters:
            self.kalman_filters[idx] = KalmanFilter()
        return self.kalman_filters[idx]

    def parse_frame(self, data):
        if len(data) < 30 or data[0] != 0x01 or data[1] != 0x03:
            return False

        try:
            if data[3:5] != b'\xca\xda':
                return False

            tag_id = struct.unpack('>H', data[7:9])[0]
            pos_idx = 13

            # Skip coordinate part if enabled
            flag = struct.unpack('>H', data[5:7])[0]
            if flag & 0x01:
                pos_idx += 6

            distances = []
            while pos_idx + 2 <= len(data) - 2:
                dist = struct.unpack('>H', data[pos_idx:pos_idx+2])[0] / 100.0
                distances.append(dist)
                pos_idx += 2

            if not distances:
                return False

            # ================== Process Distances ==================
            filtered_distances = []
            for i, d in enumerate(distances):
                if self.use_kalman and d > 0.5:          # Filter only valid distances
                    kf = self.get_kalman(i)
                    f_d = kf.update(d)
                else:
                    f_d = d
                filtered_distances.append(f_d)

            current_rounded = [round(d, 3) for d in filtered_distances]

            # De-duplication
            if (self.last_distances is None or 
                self.last_tag_id != tag_id or 
                current_rounded != self.last_distances):
                
                dist_msg = Float32MultiArray()
                dist_msg.data = filtered_distances
                self.pub_distances.publish(dist_msg)

                self.last_distances = current_rounded
                self.last_tag_id = tag_id

                # ================== Record data for plotting ==================
                if len(self.raw_data) < self.record_count:
                    # Take the first valid distance
                    valid_raw = next((d for d in distances if d > 0.5), 0)
                    valid_filt = next((d for d in filtered_distances if d > 0.5), 0)
                    
                    self.raw_data.append(valid_raw)
                    self.filtered_data.append(valid_filt)

                    if len(self.raw_data) >= self.record_count:
                        self.plot_comparison()

            return True

        except:
            return False

    def plot_comparison(self):
        plt.figure(figsize=(12, 7))
        x = range(len(self.raw_data))
        
        plt.plot(x, self.raw_data, 'r-', label='Raw Data', alpha=0.8)
        plt.plot(x, self.filtered_data, 'b-', label='Kalman Filtered', linewidth=2.5)
        
        plt.title('UWB Real Data - Kalman Filter Effect Comparison (1000 points)', fontsize=15)
        plt.xlabel('Data Index', fontsize=12)
        plt.ylabel('Distance (meters)', fontsize=12)
        plt.legend(fontsize=12)
        plt.grid(True, alpha=0.3)
        
        plt.savefig('uwb_kalman_real_test.png', dpi=200, bbox_inches='tight')
        rospy.loginfo("✅ Kalman filter comparison chart saved as: uwb_kalman_real_test.png")
        plt.show()

    def run(self):
        buffer = b''
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)

                while len(buffer) > 30:
                    if buffer[:2] == b'\x01\x03':
                        byte_count = buffer[2]
                        frame_len = 5 + byte_count + 2
                        if len(buffer) >= frame_len:
                            self.parse_frame(buffer[:frame_len])
                            buffer = buffer[frame_len:]
                        else:
                            break
                    else:
                        buffer = buffer[1:]
            rate.sleep()

if __name__ == '__main__':
    try:
        UWBParser().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Node exception: {e}")