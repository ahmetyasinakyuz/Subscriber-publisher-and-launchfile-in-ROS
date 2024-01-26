# ROS kütüphanelerini içe aktar
import rospy
from std_msgs.msg import String

# Callback fonksiyonu: Konudan alınan veriyi işler
def callback(data):
    rospy.loginfo("Alınan veri: %s", data.data)

# Listener fonksiyonu: Abonelik başlatır ve sürekli olarak veri bekler
def listener():
    rospy.init_node('listener', anonymous=True)  # ROS düğümünü başlat
    rospy.Subscriber("sensor_data", String, callback)  # "sensor_data" konusuna abone ol
    rospy.spin()  # Düğümü çalışır durumda tut

if __name__ == '__main__':
    listener()  # Listener fonksiyonunu çağır
