#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <string>
#include <iostream>
#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include "upper_body_detector/UpperBodyDetector.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <QImage>
#include <QPainter>
#include <vector>
#include "forest_utilities.h"
#include "bodypose_regressor_utilities.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "rwth_upper_body_skeleton_random_walk/SkeletonDetector.h"

unsigned counter = 0;
unsigned detection = 0;
unsigned im_counter = 0;
using namespace rwth_upper_body_skeleton_random_walk;

image_transport::Publisher pub_result_image;
ros::Publisher pub_3d_positions;
ros::ServiceClient upper_body_skeleton_client;

int joint_connections[9] = {-1,0,1,1,1,3,4,5,6};
unsigned bone_colors[8][3] = {{0,0,255},{0,0,255},{255,0,255},{255,0,0},{0,255,0},{255,255,0},{255,0,255},{255,0,0}};

FOREST f(9,640,480);




 void render_bbox_with_skeleton(QImage& image,
                    int r, int g, int b, int lineWidth,
                    std::vector<std::vector<unsigned short> > &skeletons,
                    const upper_body_detector ::UpperBodyDetector::ConstPtr &detections,
		    unsigned total_skeletons_predicted,
                    std::vector<unsigned> &upper_body_for_skeleton)
{

    QPainter painter(&image);
    QColor qColor;
    
    QPen pen;
    pen.setWidth(5);
    
    //unsigned N = total_skeletons_predicted; 
    unsigned joint_counter = 0;

    for(unsigned i = 0; i < total_skeletons_predicted; i++){

        joint_counter = 0;
        unsigned upper_body_index = upper_body_for_skeleton[i]; 
        int x =(int) detections->pos_x[upper_body_index]-5;
        int y =(int) detections->pos_y[upper_body_index]-5;
        int w =(int) detections->width[upper_body_index]+45;
        int h =(int) detections->height[upper_body_index]+100/detections->median_depth[upper_body_index];
          
        qColor.setRgb(r, g, b);
        pen.setColor(qColor);
        

        painter.setPen(pen);
         
        painter.drawLine(x,y, x+w,y);
        painter.drawLine(x,y, x,y+h);
        painter.drawLine(x+w,y, x+w,y+h);
        painter.drawLine(x,y+h, x+w,y+h);
        unsigned bone_counter = 0;
        for (unsigned j = 0 ; j < 18 ; j = j + 2)
		{
                  
		  if (joint_connections[joint_counter] >= 0)
		   { 
                     
                    		  
                     unsigned short index = joint_connections[joint_counter];	
                      if ( (skeletons[i][index*2] == 0) || (skeletons[i][index*2+1] == 0) || (skeletons[i][j] == 0) || (skeletons[i][j+1]==0) )
		        {
                           std::cout << "\ncannot predict skeleton\n";
                           break;
			} 
		     qColor.setRgb(bone_colors[bone_counter][0], bone_colors[bone_counter][1], bone_colors[bone_counter][2]);
    		     pen.setColor(qColor);
                     painter.setPen(pen);
                    // std::cout << "\n" << skeletons[i][index*2] << "," << skeletons[i][index*2+1] ; 
		     painter.drawLine(skeletons[i][index*2],skeletons[i][index*2+1], skeletons[i][j],skeletons[i][j+1]);
		     bone_counter++;	
		   }
                joint_counter++;
		
	    }
	}

}
         


void convert_to_2d(std::vector<std::vector<float> > &_3d_positions, std::vector<unsigned short> &_2d_positions)
{
		
	
        unsigned _2d_counter = 0;
    	for (unsigned j = 0 ; j < 9 ; j = j + 1)
	 {
		unsigned short c1 = 480/2;
	    	unsigned short c2 = 640/2;
              unsigned row = (530 * _3d_positions[j][0])/_3d_positions[j][2] + c2;
		unsigned col = (-530 *_3d_positions[j][1])/_3d_positions[j][2] + c1;
		_2d_positions[_2d_counter++] = row;
		_2d_positions[_2d_counter++] = col;

      }
 
}




    	

void callback(const sensor_msgs::ImageConstPtr& depth , const sensor_msgs::ImageConstPtr& color, const upper_body_detector ::UpperBodyDetector::ConstPtr &upper)
{
   
    detection = 0;
    cv_bridge:: CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(depth);
 
    const cv::Mat &im = cv_ptr->image; 
    cv::Size s = cv_ptr->image.size();
    int rows = s.height;
    int cols = s.width;
    
    counter++;
    ROS_INFO("Recieved frame %d", counter);
    
    
   
   unsigned short total_upper_bodies = upper->pos_x.size();
   std::vector<std::vector<unsigned short> > _2d_upper_body_skeletons(total_upper_bodies,std::vector<unsigned short>(18));    
   unsigned total_skeleton = 0;
   std::vector<unsigned> upper_body_for_skeleton;
  

  
   for (unsigned i =0 ; i < total_upper_bodies ; i++)
   	{ 
            SkeletonDetector sk;  
            if (upper->median_depth[i] <= 1)
            {
                 ROS_INFO("Person too close."); 
                 continue;
            }
            
    	  
            std::vector<float > bounding_box(5);    
            bounding_box[0] = upper->pos_x[i]; bounding_box[1] = upper->pos_y[i]; 
            bounding_box[2] = upper->width[i]; bounding_box[3] = upper->height[i];
            bounding_box[4] = upper->median_depth[i];

            std::vector<std::vector<float> > max_scoring_joints(9,std::vector<float>(3));
            compute_upper_body_pose(im, f, bounding_box ,max_scoring_joints);
            for (unsigned joint = 0 ; joint < 9 ; joint++)
                 for (unsigned coord = 0 ; coord < 3 ; coord++)             
                            sk.pos_3d.push_back(max_scoring_joints[joint][coord]);
            //pub_3d_positions.publish(max_scoring_joints);
            std::vector<unsigned short> _2d_positions(18);
            
            convert_to_2d(max_scoring_joints, _2d_upper_body_skeletons[i]);
            upper_body_for_skeleton.push_back(i);  
            total_skeleton++;
            pub_3d_positions.publish(sk);
            
           
           }
   
   
   
   QImage image_rgb(&color->data[0], color->width, color->height, QImage::Format_RGB888);
   render_bbox_with_skeleton(image_rgb, 0, 0, 255, 2,_2d_upper_body_skeletons,upper,total_skeleton,upper_body_for_skeleton);
   sensor_msgs::Image sensor_image;
   sensor_image.header = color->header;
   sensor_image.height = image_rgb.height();
   sensor_image.width  = image_rgb.width();
   sensor_image.step   = color->step;
   std :: vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
   sensor_image.data = image_bits;
   sensor_image.encoding = color->encoding;
   pub_result_image.publish(sensor_image);
  
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "upper_body_skeleton_detector");
  
  ros::NodeHandle n;
  //upper_body_skeleton_client = n.serviceClient<rwth_upper_body_skeleton_detector::GetUpperBodySkeleton>("/rwth_upper_body_skeleton_detector/get_upper_body_skeleton");
  image_transport::ImageTransport it(n); 
  std::string model_path;
  std::string depth_image_msg;
  std::string rgb_image_msg;
  std::string upper_body_msg;
  std::string pub_topic_pos; 
  
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("model_path", model_path,std::string(""));
  private_node_handle_.param("depth_image_msg", depth_image_msg,std::string(""));
  private_node_handle_.param("rgb_image_msg", rgb_image_msg,std::string(""));
  private_node_handle_.param("upper_body_msg", upper_body_msg,std::string(""));
  ROS_INFO("Starting upper_body_skeleton_detector....");

   //loading the forest
  f.load_forest(model_path); 


  
  image_transport::SubscriberFilter subscriber_depthimage(it,depth_image_msg, 1);
  image_transport::SubscriberFilter subscriber_rgbimage(it,rgb_image_msg,1);
  message_filters::Subscriber<upper_body_detector::UpperBodyDetector> subscriber_upperbody(n,upper_body_msg, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,upper_body_detector::UpperBodyDetector> MySyncPolicy;
 //const SyncType sync_policy(10);
  message_filters::Synchronizer< MySyncPolicy > mysync(MySyncPolicy( 10 ), subscriber_depthimage,subscriber_rgbimage ,subscriber_upperbody);
  mysync.registerCallback(boost::bind(&callback, _1, _2,_3));

  //Publisher
  pub_result_image = it.advertise("/rwth_upper_body_skeleton_random_walk/colorimage", 1);
  private_node_handle_.param("rwth_upper_body_skeleton_random_walk", pub_topic_pos, std::string("/rwth_upper_body_skeleton_random_walk/_3d_joint_positions"));
  pub_3d_positions = n.advertise<SkeletonDetector>(pub_topic_pos.c_str(),10);

  ros::spin();

  return 0;
}
