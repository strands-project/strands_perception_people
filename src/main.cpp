// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string.h>
#include <QImage>
#include <QPainter>

#include <cudaHOG.h>

#include "strands_perception_people_msgs/GroundHOGDetections.h"
#include "strands_perception_people_msgs/GroundPlane.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace strands_perception_people_msgs;
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
cudaHOG::cudaHOGManager *hog;
ros::Publisher pub_message;
ros::Publisher pub_result_image;

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

    std::vector<cudaHOG::Detection> detHog;

    //  unsigned char image
    QImage image_rgb(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    int returnPrepare = hog->prepare_image(image_rgb.convertToFormat(QImage::Format_ARGB32).bits(), (short unsigned int) msg->width, (short unsigned int) 	msg->height);

    if(returnPrepare)
    {
        ROS_ERROR("Error by preparing the image");
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
    
    render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);
    
    Image sensor_image;
    sensor_image.header = msg->header;
    sensor_image.height = image_rgb.height();
    sensor_image.width  = image_rgb.width();
    vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
    sensor_image.data = image_bits;
    sensor_image.encoding = msg->encoding;

    pub_result_image.publish(sensor_image);
    pub_message.publish(detections);
} 

void imageGroundPlaneCallback(const ImageConstPtr &color, const GroundPlaneConstPtr &gp)
{

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "groundHOG");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    string image_color;
    string ground_plane;
    string pub_topic;
    string pub_image_topic;
    string conf;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("model", conf, string(""));
    private_node_handle_.param("image_color", image_color, string("/camera/rgb/image_color"));
    private_node_handle_.param("ground_plane", ground_plane, string(""));
    private_node_handle_.param("detections", pub_topic, string("/groundHOG/detections"));
    private_node_handle_.param("result_image", pub_image_topic, string("/groundHOG/image"));


    //Initialise cudaHOG
    if(strcmp(conf.c_str(),"") == 0) {
        ROS_ERROR("No model path specified.");
        ROS_ERROR("Run with: rosrun strands_ground_hog groundHOG _model:=/path/to/model");
        exit(0);
    }
    hog = new  cudaHOG::cudaHOGManager();
    hog->read_params_file(conf);
    hog->load_svm_models();

    // Create a subscriber.
    // Name the topic, message queue, callback function with class name, and object containing callback function.
    //The bigger the queue, the bigger the dealy. 1 is the most real-time.
    ros::Subscriber sub_message; //Subscribeers have to be defined out of the if scope to have affect.
    Subscriber<GroundPlane> subscriber_ground_plane(n, ground_plane.c_str(), 1);
    Subscriber<Image> subscriber_color(n, image_color.c_str(), 1);

    if(strcmp(ground_plane.c_str(), "") == 0) {
        sub_message = n.subscribe(image_color.c_str(), 1, &imageCallback);
    } else {
        sync_policies::ApproximateTime<Image, GroundPlane> MySyncPolicy(10);
        const sync_policies::ApproximateTime<Image, GroundPlane> MyConstSyncPolicy = MySyncPolicy;
        Synchronizer< sync_policies::ApproximateTime<Image, GroundPlane> > sync(MyConstSyncPolicy,
                                                                                      subscriber_color,
                                                                                      subscriber_ground_plane);

        sync.registerCallback(boost::bind(&imageGroundPlaneCallback, _1, _2));
    }

    // Create publishers
    pub_message = n.advertise<strands_perception_people_msgs::GroundHOGDetections>(pub_topic.c_str(), 10);
    pub_result_image = n.advertise<sensor_msgs::Image>(pub_image_topic.c_str(), 10);

    ros::spin();

    return 0;
}

