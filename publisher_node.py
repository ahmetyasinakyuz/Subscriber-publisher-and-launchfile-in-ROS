#!/usr/bin/env python
# Yukarıdaki satır, bu dosyanın bir Python betiği olduğunu belirtir.

import rospy
from std_msgs.msg import String
# rospy, ROS Python kütüphanesini, String ise standart mesaj tipini içe aktarır.

rospy.init_node('publisher_node')
# ROS düğümünü başlatır ve 'publisher_node' ismini verir.

pub = rospy.Publisher('my_topic', String, queue_size=10)
# 'my_topic' isminde bir ROS topic oluşturur. String tipinde veri taşıyacak.
# 'queue_size', ileti sırasının boyutunu belirler.

rate = rospy.Rate(1)  # 1 Hz
# Publisher'ın belirli bir frekansta çalışmasını sağlar.

while not rospy.is_shutdown():
    msg = "Hello, ROS!"
    rospy.loginfo(msg)
    # 'rospy.loginfo()' ile mesajı ROS log sistemine yazdırır.
    
    pub.publish(msg)
    # Mesajı topic üzerinden yayınlar.

    rate.sleep()
    # Belirlenen frekansta çalışmak için bekleme süresi ekler.

