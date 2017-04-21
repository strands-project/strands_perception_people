// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>


#include "visual_odometry/VisualOdometry.h"

ros::Publisher pub_message;


int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "tf2visual");
    ros::NodeHandle n;

    std::string pub_topic;
    std::string sensor_frame_id;


    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");

    private_node_handle_.param("motion_parameters", pub_topic, std::string("/visual_odometry/motion_matrix"));
    private_node_handle_.param("sensor_frame_id", sensor_frame_id, std::string("head_xtion_rgb_optical_frame"));
    pub_message = n.advertise<visual_odometry::VisualOdometry>(pub_topic.c_str(), 3);

    tf::TransformListener listener;

    ros::Rate updateRate(30.0);
    while(ros::ok()) {
        ros::spinOnce();
        updateRate.sleep();

        tf::StampedTransform transform;
        try{
          listener.lookupTransform(sensor_frame_id, "odom", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
          ROS_WARN_THROTTLE(10.0, "Failed to lookup transform in tf2visual: %s",ex.what());
        }

        Eigen::Affine3d finalTransform;
        tf::transformTFToEigen(transform, finalTransform);

        Eigen::Matrix4d ros2tracker_R;
        ros2tracker_R << 0,-1,0,0, 0,0,-1,0, 1,0,0,0, 0,0,0,1;

        Eigen::Matrix4d finalMatrix = finalTransform.matrix().inverse();

        Eigen::Matrix4d rotatedFinalMatrix = (ros2tracker_R*finalMatrix);

        // and mirror at x-z-plane (turns a +0 in a -0 in t and does sth. to R, but still...)
        /*Eigen::Affine3d mirrorTransform;
        mirrorTransform.setIdentity();
        Eigen::Matrix3d A_rh = mirrorTransform.linear();
        Eigen::Vector3d b_rh = mirrorTransform.translation();

        Eigen::Matrix3d S_y; S_y << 1, 0, 0,   0, -1, 0,  0, 0, 1;

        Eigen::Matrix3d A_lh = S_y * A_rh * S_y;
        Eigen::Vector3d b_lh = S_y * b_rh;

        Eigen::Affine3d finalMirrorTransform;
        finalMirrorTransform.linear() = A_lh;
        finalMirrorTransform.translation() = b_lh;*/

        //Eigen::Matrix4d reallyFinalMatrix = (finalMirrorTransform*rotatedFinalMatrix).matrix().transpose();
        Eigen::Matrix4d reallyFinalMatrix = rotatedFinalMatrix.transpose();

        visual_odometry::VisualOdometry fovis_info_msg;
        fovis_info_msg.header.frame_id = sensor_frame_id;
        fovis_info_msg.header.stamp = transform.stamp_;
        fovis_info_msg.motion_estimate_valid = true;
        fovis_info_msg.transformation_matrix.resize(4*4);
        for(int i = 0; i < 4*4; i++)
            fovis_info_msg.transformation_matrix[i] = reallyFinalMatrix.data()[i];
         pub_message.publish(fovis_info_msg);
    }


    return 0;
}
