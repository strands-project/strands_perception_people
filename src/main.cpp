// ROS includes.
#include <ros/ros.h>

#if WITH_CUDA
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <XmlRpc.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <string.h>
#include <sstream>
#include <glib.h>
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
image_transport::Publisher pub_result_image;

bool checkParam(bool success, std::string param) {
    if(!success) {
        ROS_FATAL("Parameter: '%s' could not be found! Please make sure that the parameters are available on the parameter server or start with 'load_params_from_file:=true'", param.c_str());
    }
    return success;
}

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

    if(pub_result_image.getNumSubscribers()) {
        ROS_DEBUG("Publishing image");
        render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);

        Image sensor_image;
        sensor_image.header = msg->header;
        sensor_image.height = image_rgb.height();
        sensor_image.width  = image_rgb.width();
        sensor_image.step   = msg->step;
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

    if(pub_result_image.getNumSubscribers()) {
        ROS_DEBUG("Publishing image");
        render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);

        Image sensor_image;
        sensor_image.header = color->header;
        sensor_image.height = image_rgb.height();
        sensor_image.width  = image_rgb.width();
        sensor_image.step   = color->step;
        vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
        sensor_image.data = image_bits;
        sensor_image.encoding = color->encoding;

        pub_result_image.publish(sensor_image);
    }

    pub_message.publish(detections);

}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(ros::Subscriber &sub_msg,
                     ros::NodeHandle &n,
                     string gp_topic,
                     string img_topic,
                     Subscriber<GroundPlane> &sub_gp,
                     Subscriber<CameraInfo> &sub_cam,
                     image_transport::SubscriberFilter &sub_col,
                     image_transport::ImageTransport &it){
    if(!pub_message.getNumSubscribers() && !pub_result_image.getNumSubscribers()) {
        ROS_DEBUG("HOG: No subscribers. Unsubscribing.");
        sub_msg.shutdown();
        sub_gp.unsubscribe();
        sub_cam.unsubscribe();
        sub_col.unsubscribe();
    } else {
        ROS_DEBUG("HOG: New subscribers. Subscribing.");
        if(strcmp(gp_topic.c_str(), "") == 0) {
            sub_msg = n.subscribe(img_topic.c_str(), 1, &imageCallback);
        }
        sub_cam.subscribe();
        sub_gp.subscribe();
        sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "ground_hog");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string ground_plane;
    string camera_ns;
    string pub_topic;
    string pub_image_topic;
    string model_path;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(10));

    private_node_handle_.param("camera_namespace", camera_ns, string("/head_xtion"));
    private_node_handle_.param("ground_plane", ground_plane, string(""));

    string image_color = camera_ns + "/rgb/image_rect_color";
    string camera_info = camera_ns + "/depth/camera_info";

    int hog_descriptor_height, hog_descritpor_width, hog_window_height, hog_window_width;
    string model_name;
    XmlRpc::XmlRpcValue model;
    unsigned char* svm_model;

    bool success = true;
    success = checkParam(private_node_handle_.getParam("hog_descriptor_height", hog_descriptor_height), "hog_descriptor_height") && success;
    success = checkParam(private_node_handle_.getParam("hog_descritpor_width", hog_descritpor_width), "hog_descritpor_width") && success;
    success = checkParam(private_node_handle_.getParam("hog_window_height", hog_window_height), "hog_window_height") && success;
    success = checkParam(private_node_handle_.getParam("hog_window_width", hog_window_width), "hog_window_width") && success;
    success = checkParam(private_node_handle_.getParam("name", model_name), "name") && success;
    success = checkParam(private_node_handle_.getParam("model", model), "model") && success;
    success = checkParam(private_node_handle_.getParam("model_path", model_path), "model_path") && success;
    if(!success) return 1;

    try {
        std::stringstream ss;
        ROS_ASSERT(model.getType() == XmlRpc::XmlRpcValue::TypeBase64); // This is taken from the example for lists but it never fails regardles of the type.
        ss << model; // Not nice but the only way I found to get a char array from the model.
        long unsigned int len;
        svm_model = g_base64_decode_inplace(const_cast<char*>(ss.str().c_str()), &len);
        std::string save_path = model_path; save_path += "/"; save_path += model_name;
        FILE* file = fopen(save_path.c_str(), "wb");
        fwrite(svm_model, sizeof(char), len, file);
        fclose(file);
    } catch (XmlRpc::XmlRpcException &e) {
        ROS_ERROR("%s", e.getMessage().c_str());
    }

    ROS_DEBUG("groundHOG: Queue size for synchronisation is set to: %i", queue_size);

    hog = new  cudaHOG::cudaHOGManager();
    hog->set_params(model_name, model_path, hog_window_width, hog_window_height, hog_descritpor_width, hog_descriptor_height);
    hog->load_svm_models();

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Name the topic, message queue, callback function with class name, and object containing callback function.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    ros::Subscriber sub_message; //Subscribers have to be defined out of the if scope to have affect.
    Subscriber<GroundPlane> subscriber_ground_plane(n, ground_plane.c_str(), 1); subscriber_ground_plane.unsubscribe();
    image_transport::SubscriberFilter subscriber_color;
    subscriber_color.subscribe(it, image_color.c_str(), 1); subscriber_color.unsubscribe();
    Subscriber<CameraInfo> subscriber_camera_info(n, camera_info.c_str(), 1); subscriber_camera_info.unsubscribe();

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(sub_message),
                                                       boost::ref(n),
                                                       ground_plane,
                                                       image_color,
                                                       boost::ref(subscriber_ground_plane),
                                                       boost::ref(subscriber_camera_info),
                                                       boost::ref(subscriber_color),
                                                       boost::ref(it));

    image_transport::SubscriberStatusCallback image_cb = boost::bind(&connectCallback,
                                                                   boost::ref(sub_message),
                                                                   boost::ref(n),
                                                                   ground_plane,
                                                                   image_color,
                                                                   boost::ref(subscriber_ground_plane),
                                                                   boost::ref(subscriber_camera_info),
                                                                   boost::ref(subscriber_color),
                                                                   boost::ref(it));

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
    pub_message = n.advertise<strands_perception_people_msgs::GroundHOGDetections>(pub_topic.c_str(), 10, con_cb, con_cb);

    private_node_handle_.param("result_image", pub_image_topic, string("/groundHOG/image"));
    pub_result_image = it.advertise(pub_image_topic.c_str(), 1, image_cb, image_cb);

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

