// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/Image.h"

#include "string.h"
#include <QImage>
#include <QPainter>

#include "cudaHOG.h"
#include "strands_perception_people_msgs/Detections.h"

using namespace std;
/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
cudaHOG::cudaHOGManager *hog;
ros::Publisher pub_message;
ros::Publisher pub_result_image;

void render_bbox_2D(strands_perception_people_msgs::Detections& detections, QImage& image, int r, int g, int b, int lineWidth)
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

void messageCallback(const sensor_msgs::Image::ConstPtr &msg)
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
   
   strands_perception_people_msgs::Detections detections;
  
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
    
    sensor_msgs::Image sensor_image;
    sensor_image.header = msg->header;
    sensor_image.height = image_rgb.height();
    sensor_image.width  = image_rgb.width();
	vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
	sensor_image.data = image_bits;
	sensor_image.encoding = msg->encoding;

    pub_result_image.publish(sensor_image);
    pub_message.publish(detections);
} 

int main(int argc, char **argv)
{
  hog = new  cudaHOG::cudaHOGManager();
  string conf = "model/config";
  hog->read_params_file(conf);
  hog->load_svm_models();
        
  // Set up ROS.
  ros::init(argc, argv, "groundHOG");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string topic;
  string pub_topic;
  string pub_image_topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(40));
  private_node_handle_.param("topic", topic, string("/camera/rgb/image_color"));

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe(topic.c_str(), 50, &messageCallback);
  
  private_node_handle_.param("topic", pub_topic, string("/groundHOG/detections"));
  pub_message = n.advertise<strands_perception_people_msgs::Detections>(pub_topic.c_str(), 10);
  
  private_node_handle_.param("topic", pub_image_topic, string("/groundHOG/image"));
  pub_result_image = n.advertise<sensor_msgs::Image>(pub_image_topic.c_str(), 10);
  
  ros::spin();
  
  return 0;
}

