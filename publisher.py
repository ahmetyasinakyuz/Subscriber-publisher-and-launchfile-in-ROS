# ROS kütüphanelerini içe aktar
import rospy
from std_msgs.msg import String
import time

# Publisher fonksiyonu: Belirli bir konuda sürekli olarak veri yayınlar
def publisher():
    rospy.init_node('publisher', anonymous=True)  # ROS düğümünü başlat
    pub = rospy.Publisher("sensor_data", String, queue_size=10)  # "sensor_data" konusunda yayın yapacak bir yayıncı oluştur
    rate = rospy.Rate(1)  # 1 Hz frekansında çalışacak bir hız nesnesi oluştur

    while not rospy.is_shutdown():
        data = "Sensör verisi: {}".format(time.time())  # Zaman bilgisini içeren bir veri oluştur
        rospy.loginfo(data)  # Veriyi ekrana yazdır
        pub.publish(data)  # Veriyi "sensor_data" konusuna yayınla
        rate.sleep()  # Belirlenen hızda bekleyerek döngüyü devam ettir

if __name__ == '__main__':
    try:
        publisher()  # Publisher fonksiyonunu çağır
    except rospy.ROSInterruptException:
        pass
