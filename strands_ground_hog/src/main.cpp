// ROS includes.
#include <ros/ros.h>

#if WITH_CUDA
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CameraInfo.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <string.h>
#include <QImage>
#include <QPainter>


#include <cudaHOG.h>

#include "strands_perception_people_msgs/GroundHOGDetections.h"
#include "strands_perception_people_msgs/GroundPlane.h"

#include "Matrix.h"
#include "Vector.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace strands_perception_people_msgs;



cudaHOG::cudaHOGManager *hog;
ros::Publisher pub_message;
ros::Publisher pub_result_image;

bool visualise;

void render_bbox_2D(GroundHOGDetections& detections, QImage& image, int r, int g, int b, int lineWidth)
{

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(lineWidth);

    painter.setPen(pen);

    for(int i = 0; i < detections.pos_x.size(); i++){
        int x =(int) detections.pos_x[i];
        int y =(int) detections.pos_y[i];
        int w =(int) detections.width[i];
        int h =(int) detections.height[i];

        painter.drawLine(x,y, x+w,y);
        painter.drawLine(x,y, x,y+h);
        painter.drawLine(x+w,y, x+w,y+h);
        painter.drawLine(x,y+h, x+w,y+h);
    }
}

void imageCallback(const Image::ConstPtr &msg)
{
    //    ROS_INFO("Entered img callback");
    std::vector<cudaHOG::Detection> detHog;

    //  unsigned char image
    QImage image_rgb(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    int returnPrepare = hog->prepare_image(image_rgb.convertToFormat(QImage::Format_ARGB32).bits(), (short unsigned int) msg->width, (short unsigned int) 	msg->height);

    if(returnPrepare)
    {
        ROS_ERROR("groundHOG: Error while preparing the image");
        return;
    }

    hog->test_image(detHog);
    hog->release_image();

    int w = 64, h = 128;

    GroundHOGDetections detections;

    detections.header = msg->header;
    for(unsigned int i=0;i<detHog.size();i++)
    {

        float scale = detHog[i].scale;

        float width = (w - 32.0f)*scale;
        float height = (h - 32.0f)*scale;
        float x = (detHog[i].x + 16.0f*scale);
        float y = (detHog[i].y + 16.0f*scale);

        detections.scale.push_back(detHog[i].scale);
        detections.score.push_back(detHog[i].score);
        detections.pos_x.push_back(x);
        detections.pos_y.push_back(y);
        detections.width.push_back(width);
        detections.height.push_back(height);

    }

    if(visualise) {
        render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);

        Image sensor_image;
        sensor_image.header = msg->header;
        sensor_image.height = image_rgb.height();
        sensor_image.width  = image_rgb.width();
        vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
        sensor_image.data = image_bits;
        sensor_image.encoding = msg->encoding;

        pub_result_image.publish(sensor_image);
    }

    pub_message.publish(detections);
}

void imageGroundPlaneCallback(const ImageConstPtr &color, const CameraInfoConstPtr &camera_info,
                              const GroundPlaneConstPtr &gp)
{
    //    ROS_INFO("Entered gp-img callback");
    std::vector<cudaHOG::Detection> detHog;

    //  unsigned char image
    QImage image_rgb(&color->data[0], color->width, color->height, QImage::Format_RGB888);
    int returnPrepare = hog->prepare_image(image_rgb.convertToFormat(QImage::Format_ARGB32).bits(),
                                           (short unsigned int) color->width, (short unsigned int) color->height);

    if(returnPrepare)
    {
        ROS_ERROR("Error by preparing the image");
        return;
    }

    // Generate base camera
    Matrix<float> R = Eye<float>(3);
    Vector<float> t(3, 0.0);
    // Get GP
    Vector<double> GPN(3, (double*) &gp->n[0]);
    double GPd = ((double) gp->d)*(-1000.0);
    Matrix<double> K(3,3, (double*)&camera_info->K[0]);

    Vector<float> float_GPN(3);
    float_GPN(0) = float(GPN(0));
    float_GPN(1) = float(GPN(1));
    float_GPN(2) = float(GPN(2));

    float float_GPd = (float) GPd;
    Matrix<float> float_K(3,3);
    float_K(0,0) = K(0,0); float_K(1,0) = K(1,0); float_K(2,0) = K(2,0);
    float_K(1,1) = K(1,1); float_K(0,1) = K(0,1); float_K(2,1) = K(2,1);
    float_K(2,2) = K(2,2); float_K(0,2) = K(0,2); float_K(1,2) = K(1,2);


    //    float_K.Show();
    //    float_GPN.show();
    //    printf("%f\n", float_GPd)

    try
    {
        hog->set_camera(R.data(), float_K.data(), t.data());
        hog->set_groundplane(float_GPN.data(), &float_GPd);
        hog->prepare_roi_by_groundplane();
        hog->test_image(detHog);
        hog->release_image();
    }
    catch(...)
    {
        ROS_WARN("GroundHOG: Extracted Ground Plane can not be used for computation of ROIs");
    }



    int w = 64, h = 128;

    GroundHOGDetections detections;

    detections.header = color->header;
    for(unsigned int i=0;i<detHog.size();i++)
    {

        float scale = detHog[i].scale;

        float width = (w - 32.0f)*scale;
        float height = (h - 32.0f)*scale;
        float x = (detHog[i].x + 16.0f*scale);
        float y = (detHog[i].y + 16.0f*scale);

        detections.scale.push_back(detHog[i].scale);
        detections.score.push_back(detHog[i].score);
        detections.pos_x.push_back(x);
        detections.pos_y.push_back(y);
        detections.width.push_back(width);
        detections.height.push_back(height);

    }

    if(visualise) {
        render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);

        Image sensor_image;
        sensor_image.header = color->header;
        sensor_image.height = image_rgb.height();
        sensor_image.width  = image_rgb.width();
        vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
        sensor_image.data = image_bits;
        sensor_image.encoding = color->encoding;

        pub_result_image.publish(sensor_image);
    }

    pub_message.publish(detections);

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "groundHOG");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string image_color;
    string ground_plane;
    string camera_info;
    string pub_topic;
    string pub_image_topic;
    string conf;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(10));
    private_node_handle_.param("model", conf, string(""));
    private_node_handle_.param("visualise", visualise, bool(false));

    private_node_handle_.param("color_image", image_color, string("/camera/rgb/image_color"));
    private_node_handle_.param("camera_info", camera_info, string("/camera/rgb/camera_info"));
    private_node_handle_.param("ground_plane", ground_plane, string(""));

    //Initialise cudaHOG
    if(strcmp(conf.c_str(),"") == 0) {
        ROS_ERROR("No model path specified.");
        ROS_ERROR("Run with: rosrun strands_ground_hog groundHOG _model:=/path/to/model");
        exit(0);
    }

    ROS_DEBUG("groundHOG: Queue size for synchronisation is set to: %i", queue_size);

    hog = new  cudaHOG::cudaHOGManager();
    hog->read_params_file(conf);
    hog->load_svm_models();

    // Create a subscriber.
    // Name the topic, message queue, callback function with class name, and object containing callback function.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    ros::Subscriber sub_message; //Subscribers have to be defined out of the if scope to have affect.
    Subscriber<GroundPlane> subscriber_ground_plane(n, ground_plane.c_str(), 1);
    Subscriber<Image> subscriber_color(n, image_color.c_str(), 1);
    Subscriber<CameraInfo> subscriber_camera_info(n, camera_info.c_str(), 1);

    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    const sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane> MyConstSyncPolicy = MySyncPolicy;
    Synchronizer< sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane> > sync(MyConstSyncPolicy,
                                                                                        subscriber_color,
                                                                                        subscriber_camera_info,
                                                                                        subscriber_ground_plane);

    // Decide which call back should be used.
    if(strcmp(ground_plane.c_str(), "") == 0) {
        sub_message = n.subscribe(image_color.c_str(), 1, &imageCallback);
    } else {
        sync.registerCallback(boost::bind(&imageGroundPlaneCallback, _1, _2, _3));
    }

    // Create publishers
    private_node_handle_.param("detections", pub_topic, string("/groundHOG/detections"));
    pub_message = n.advertise<strands_perception_people_msgs::GroundHOGDetections>(pub_topic.c_str(), 10);

    if(visualise) {
        private_node_handle_.param("result_image", pub_image_topic, string("/groundHOG/image"));
        pub_result_image = n.advertise<sensor_msgs::Image>(pub_image_topic.c_str(), 10);
    }

    ros::spin();

    return 0;
}

#else

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "groundHOG");
    ros::NodeHandle n;
    ROS_ERROR("strands_ground_hog package has been compiled without libcudaHOG.");
    ROS_ERROR("Please see strands_perception_people/3rd_party for instructions.");
    ROS_ERROR("This node will have no functionality unless compiled with cuda support.");
}

#endif

