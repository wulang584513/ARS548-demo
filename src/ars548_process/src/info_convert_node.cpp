#include <ros/ros.h>

#include <ars548_msg/objects.h>
#include <ars548_msg/ObjectList.h>
#include <ars548_msg/detections.h>
#include <ars548_msg/DetectionList.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

ros::Publisher objects_marker_pub;
ros::Publisher detections_cloud_pub;

void objectReceive(const ars548_msg::ObjectList& msg)
{
    uint size = msg.object_array.size();
    visualization_msgs::Marker my_marker; 
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();

    if(size>0)
    {
        for(uint i=0; i<size; i++)
        {
            my_marker.header.frame_id = "/world";
            my_marker.header.stamp = msg.object_array[i].header.stamp;
            my_marker.ns = "object_shapes";
            
            my_marker.id = msg.object_array[i].u_ID; 
            my_marker.type = visualization_msgs::Marker::CUBE;
            my_marker.action = visualization_msgs::Marker::ADD; 
            my_marker.pose.position.x = msg.object_array[i].u_Position_X;
            my_marker.pose.position.y = msg.object_array[i].u_Position_Y;
            my_marker.pose.position.z = msg.object_array[i].u_Position_Z;

            my_marker.pose.orientation.x = 0.0;
            my_marker.pose.orientation.y = 0.0;
            my_marker.pose.orientation.z = sin(msg.object_array[i].u_Position_Orientation/2);
            my_marker.pose.orientation.w = cos(msg.object_array[i].u_Position_Orientation/2);

            if((msg.object_array[i].u_Shape_Length_Edge_Mean>0.2)||(msg.object_array[i].u_Shape_Width_Edge_Mean>0.2))
            {
                my_marker.scale.x = msg.object_array[i].u_Shape_Length_Edge_Mean;
                my_marker.scale.y = msg.object_array[i].u_Shape_Width_Edge_Mean;
                my_marker.scale.z = (msg.object_array[i].u_Shape_Length_Edge_Mean+msg.object_array[i].u_Shape_Width_Edge_Mean)/2;
            }
            else
            {
                my_marker.scale.x = 0.2;
                my_marker.scale.y = 0.2;
                my_marker.scale.z = 0.2;
            }

            my_marker.color.r = 0.0f;
            my_marker.color.g = 1.0f;
            my_marker.color.b = 0.0f;
            my_marker.color.a = 1.0;

            my_marker.lifetime = ros::Duration(0.5);

            marker_array.markers.push_back(my_marker);
        }

        objects_marker_pub.publish(marker_array);
    }
}

void detectionReceive(const ars548_msg::DetectionList& msg)
{
    uint size = msg.detection_array.size();

    sensor_msgs::PointCloud cloud;  

    geometry_msgs::Point32 p;

    if(size>0)
    {
        cloud.header.frame_id = "/world";
        cloud.header.stamp = msg.detection_array[0].header.stamp;
        cloud.points.clear();
        for(uint i=0;i<size;i++) 
        {
            p.x = msg.detection_array[i].f_x;
            p.y = msg.detection_array[i].f_y; 
            p.z = msg.detection_array[i].f_z; 

            cloud.points.push_back(p);
        }

        detections_cloud_pub.publish(cloud);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_convert_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber obj_list_sub = nh.subscribe("/ars548_process/object_list", 10, &objectReceive);
    ros::Subscriber det_list_sub = nh.subscribe("/ars548_process/detection_list", 10, &detectionReceive);
    objects_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/ars548_process/object_marker", 10);
    detections_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/ars548_process/detection_point_cloud", 10);

    ros::spin();
}