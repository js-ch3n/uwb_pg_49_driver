#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np

class BiasTester:
    def __init__(self):
        rospy.init_node('uwb_bias_tester', anonymous=True)
        
        self.count = 0
        self.max_samples = 1000
        self.distances = []          # 存所有原始数据
        
        self.sub = rospy.Subscriber('/uwb/distance', Float32, self.callback)
        
        rospy.loginfo("零偏测试节点已启动 | 正在采集第一个基站的1000个数据...")

    def callback(self, msg):
        if self.count >= self.max_samples:
            return
            
        dist = msg.data
        
        if dist > 0.1:                     # 过滤无效值
            self.distances.append(dist)
            self.count += 1
            
            if self.count % 100 == 0:
                rospy.loginfo(f"已采集 {self.count}/{self.max_samples} 个样本...")
        
        # 采集完成
        if self.count >= self.max_samples:
            self.save_and_plot()
            rospy.signal_shutdown("测试完成")

    def save_and_plot(self):
        if not self.distances:
            rospy.logerr("没有采集到有效数据")
            return
            
        avg = sum(self.distances) / len(self.distances)
        
        rospy.loginfo("========== 测试完成 ==========")
        rospy.loginfo(f"第一个基站平均值（零偏参考）: {avg:.4f} m")
        
        # 保存为 txt 文件
        with open('uwb_bias_data.txt', 'w') as f:
            f.write("Sample_Index,Distance(m)\n")
            for i, d in enumerate(self.distances):
                f.write(f"{i},{d:.4f}\n")
        
        rospy.loginfo("数据已保存到: uwb_bias_data.txt")
        
        # 画折线图
        plt.figure(figsize=(10, 6))
        plt.plot(self.distances, 'b-', linewidth=0.8)
        plt.axhline(y=avg, color='r', linestyle='--', label=f'平均值 = {avg:.4f} m')
        plt.title('UWB 第一个基站测距值 - 1000样本')
        plt.xlabel('样本序号')
        plt.ylabel('距离 (m)')
        plt.grid(True)
        plt.legend()
        
        plt.savefig('uwb_bias_plot.png', dpi=300, bbox_inches='tight')
        rospy.loginfo("折线图已保存为: uwb_bias_plot.png")
        # plt.show()

if __name__ == '__main__':
    try:
        tester = BiasTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass