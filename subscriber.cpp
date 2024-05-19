#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

// Özel bir PointType tanımlayın
struct PointXYZAngle
{
    PCL_ADD_POINT4D;  // X, Y, Z koordinatlarını içerir
    float angle;      // Açısal bilgi
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Eigen için hizalama operatörü
} EIGEN_ALIGN16;      // 16 bayt hizalama

// PCL'nin bu özel PointType'ı tanıması için gerekli makrolar
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZAngle,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, angle, angle))

typedef pcl::PointCloud<PointXYZAngle> PointCloud;

class FrontVision
{
private:
    ros::Subscriber sub;
    ros::Publisher pub;
    float change_threshold; // Büyük değişiklik eşiği
    PointXYZAngle last_point; // Son nokta

public:
    FrontVision(ros::NodeHandle &nh) : change_threshold(6)
    {
        sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &FrontVision::pointCloudCallback, this);
        pub = nh.advertise<PointCloud>("frontvision", 100000);

        // İlk noktayı başlangıç olarak kabul edelim
        last_point.x = 0.0;
        last_point.y = 0.0;
        last_point.z = 0.0;
        last_point.angle = 0.0;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        PointCloud::Ptr front_vision(new PointCloud);
        front_vision->header.frame_id = "velodyne";
        front_vision->height = front_vision->width = 1;

        std::vector<uint8_t> data = msg->data;
        int point_step = msg->point_step;

        ROS_INFO("Size of point cloud data: %ld, %ld \n", data.size() / msg->point_step, msg->width);

        for (size_t i = 0; i < data.size(); i += point_step)
        {
            float x, y, z;
            memcpy(&x, &data[i], sizeof(float));
            memcpy(&y, &data[i + sizeof(float)], sizeof(float));
            memcpy(&z, &data[i + 2 * sizeof(float)], sizeof(float));

            // Açısal hesaplamalar
            float angle_horizontal = atan2(y, x); // Yatay açı

            if ((angle_horizontal > 2.0 && angle_horizontal < 4.0) || (angle_horizontal > -4.0 && angle_horizontal < -2.0))
                continue;
           

            PointXYZAngle point;
            PointXYZAngle tmpPoint;
            if (z - last_point.z > 0.0)
            {
                point.x = x;
                point.y = y;
                point.z = z;
                last_point = point; // Son noktayı güncelle
            }
            else if (std::fabs(z - last_point.z) > 0.08)
            {
                for (size_t j = 0; j < point_step - 1; j++)
                {
                    if (std::fabs(x - last_point.x) > 2 || std::fabs(y - last_point.y) > 2 || std::fabs(z - last_point.z) < 0.08)
                        break ;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    last_point = point;
                }
            }
            else
            {
                tmpPoint.x = x;
                tmpPoint.y = y;
                tmpPoint.z = z;
                last_point = tmpPoint;
            }

            point.angle = angle_horizontal; // Yatay açıyı angle alanında sakla
            front_vision->points.push_back(point);
        }

        front_vision->width = front_vision->points.size();
        front_vision->height = msg->height;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*front_vision, output);

        output.header = msg->header;
        pub.publish(output);

        // Her bir noktanın X, Y, Z ve Angle değerlerini yazdır
        for (const auto& point : front_vision->points)
        {
            ROS_INFO("X=%f, Y=%f, Z=%f, Angle=%f", point.x, point.y, point.z, point.angle);
        }
    }

    void setChangeThreshold(float threshold)
    {
        change_threshold = threshold;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "front_vision");
    ros::NodeHandle nh;
    FrontVision front = FrontVision(nh);
    // Eşik değerini değiştirmek için bu fonksiyonu kullanabilirsiniz
    // front.setChangeThreshold(0.5);
    ros::spin();
}
