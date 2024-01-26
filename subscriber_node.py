#!/usr/bin/env python
# Yukarıdaki satır, bu dosyanın bir Python betiği olduğunu belirtir.

import rospy
from std_msgs.msg import String
# rospy, ROS Python kütüphanesini, String ise standart mesaj tipini içe aktarır.

def callback(data):
    rospy.loginfo("Received: %s", data.data)
    # Yeni bir mesaj alındığında çalışacak olan fonksiyon.
    # Alınan mesajı ROS log sistemine yazdırır.

rospy.init_node('subscriber_node')
# ROS düğümünü başlatır ve 'subscriber_node' ismini verir.

rospy.Subscriber('my_topic', String, callback)
# 'my_topic' isimli ROS topic üzerinde dinleme yapacak bir Subscriber oluşturur.
# Gelen mesajları 'callback' fonksiyonuna iletilir.

rospy.spin()
# Programın sonlanmamasını sağlar. Subscriber'ın mesajları dinlemeye devam etmesini sağlar.

