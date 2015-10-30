#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <bayes_people_tracker/PeopleTracker.h>
#include <visualization_msgs/MarkerArray.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <bayes_people_tracker/people_marker.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/MapMetaData.h>

ros::Publisher pt_pub, ma_pub, ps_pub, pa_pub, p_pub;
nav_msgs::OccupancyGrid ppl_map;
PeopleMarker pm;

typedef uint32_t index_t;
typedef int16_t coord_t;

struct Cell
{
  Cell(const coord_t x=0, const coord_t y=0) : x(x), y(y) {}
  coord_t x;
  coord_t y;

  bool operator== (const Cell& c) const;
  bool operator< (const Cell& c) const;
};

inline
tf::Transform mapToWorld (const nav_msgs::MapMetaData& info)
{
  tf::Transform world_to_map;
  tf::poseMsgToTF (info.origin, world_to_map);
  return world_to_map;
}

inline
tf::Transform worldToMap (const nav_msgs::MapMetaData& info)
{
  return mapToWorld(info).inverse();
}

inline
bool withinBounds (const nav_msgs::MapMetaData& info, const Cell& c)
{
  return (c.x >= 0) && (c.y >= 0) && (c.x < (coord_t) info.width) && (c.y < (coord_t) info.height);
}

inline
Cell pointCell (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
  tf::Point pt;
  tf::pointMsgToTF(p, pt);
  tf::Point p2 = worldToMap(info)*pt;
  return Cell(floor(p2.x()/info.resolution), floor(p2.y()/info.resolution));
}

struct CellOutOfBoundsException
{
  CellOutOfBoundsException () {}
};

inline
index_t cellIndex (const nav_msgs::MapMetaData& info, const Cell& c)
{
  if (!withinBounds(info, c))
    throw CellOutOfBoundsException();
  return c.x + c.y*info.width;
}

inline
index_t pointIndex (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
  return cellIndex(info, pointCell(info, p));
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ppl_map = *msg;
}

void callback(const bayes_people_tracker::PeopleTracker::ConstPtr& pt,
              const geometry_msgs::PoseStamped::ConstPtr& ps,
              const geometry_msgs::PoseArray::ConstPtr& pa,
              const people_msgs::People::ConstPtr& p) {
    if(pt->poses.size() != 0){ // Check if there are detections
        // Create new messages
        bayes_people_tracker::PeopleTracker pt_out;
        pt_out.header = pt->header;
        geometry_msgs::PoseStamped ps_out;
        ps_out.header = ps->header;
        geometry_msgs::PoseArray pa_out;
        pa_out.header = pa->header;
        people_msgs::People p_out;
        p_out.header = p->header;

        for(int i = 0; i < pt->poses.size(); i++){
            try {
                if(int(ppl_map.data.at(pointIndex(ppl_map.info, pt->poses[i].position))) == 0) { // Check if detection is in allowed area
                    pt_out.distances.push_back(pt->distances[i]);
                    pt_out.angles.push_back(pt->angles[i]);
                    pt_out.poses.push_back(pt->poses[i]);
                    pt_out.uuids.push_back(pt->uuids[i]);
                    pt_out.velocities.push_back(pt->velocities[i]);

                    p_out.people.push_back(p->people[i]); // Both arrays are in the same order.
                }
            } catch (CellOutOfBoundsException) {
                ROS_WARN("Cell out of bounds");
                continue;
            }
        }
        if(pt_out.distances.size() != 0){
            pt_out.min_distance = *std::min_element(pt_out.distances.begin(), pt_out.distances.end());
            int idx = std::distance(pt_out.distances.begin(), std::min_element(pt_out.distances.begin(), pt_out.distances.end()));
            pt_out.min_distance_angle = pt_out.angles[idx];
            ps_out.pose = pt_out.poses[idx];
        }
        pt_pub.publish(pt_out);
        p_pub.publish(p_out);
        ps_pub.publish(ps_out);

        pa_out.poses = pt_out.poses;
        pa_pub.publish(pa_out);

        // Generate new markers
        if(ma_pub.getNumSubscribers()) {
            visualization_msgs::MarkerArray marker_array;
            for(int i = 0; i < pt_out.poses.size(); i++) {
                std::vector<visualization_msgs::Marker> human = pm.createHuman(i*10, pt_out.poses[i], pt_out.header.frame_id);
                marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
            }
            ma_pub.publish(marker_array);
        }
    } else {
        pt_pub.publish(pt);
        visualization_msgs::MarkerArray marker_array;
        ma_pub.publish(marker_array);
        ps_pub.publish(ps);
        pa_pub.publish(pa);
        p_pub.publish(p);
    }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(message_filters::Subscriber<bayes_people_tracker::PeopleTracker> &pt_sub,
                     message_filters::Subscriber<geometry_msgs::PoseStamped> &ps_sub,
                     message_filters::Subscriber<geometry_msgs::PoseArray> &pa_sub,
                     message_filters::Subscriber<people_msgs::People> &p_sub){
    if(!pt_pub.getNumSubscribers() && !ma_pub.getNumSubscribers() && !ps_pub.getNumSubscribers() && !pa_pub.getNumSubscribers() && !p_pub.getNumSubscribers()) {
        ROS_DEBUG("Tracker filter: No subscribers. Unsubscribing.");
        pt_sub.unsubscribe();
        ps_sub.unsubscribe();
        pa_sub.unsubscribe();
        p_sub.unsubscribe();
    } else {
        ROS_DEBUG("Tracker filter: New subscribers. Subscribing.");
        pt_sub.subscribe();
        ps_sub.subscribe();
        pa_sub.subscribe();
        p_sub.subscribe();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_tracker_filter");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string map_topic;
    pn.param("map_topic", map_topic, std::string("/ppl_filter_map"));
    ros::Subscriber sub = n.subscribe(map_topic, 1, map_callback);
    while(ppl_map.header.stamp.sec == 0 and ros::ok()){
        ROS_INFO_NAMED(n.getNamespace(), "Waiting for map");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
    if(ppl_map.header.stamp.sec != 0) ROS_INFO_NAMED(n.getNamespace(), "Got map");

    std::string sub_positions_topic, sub_pose_topic, sub_pose_array_topic, sub_people_topic;
    pn.param("positions", sub_positions_topic, std::string("/people_tracker/positions"));
    pn.param("pose", sub_pose_topic, std::string("/people_tracker/pose"));
    pn.param("pose_array", sub_pose_array_topic, std::string("/people_tracker/pose_array"));
    pn.param("people", sub_people_topic, std::string("/people_tracker/people"));
    message_filters::Subscriber<bayes_people_tracker::PeopleTracker> pt_sub(n, sub_positions_topic, 1);    pt_sub.unsubscribe();
    message_filters::Subscriber<geometry_msgs::PoseStamped>          ps_sub(n, sub_pose_topic, 1);         ps_sub.unsubscribe();
    message_filters::Subscriber<geometry_msgs::PoseArray>            pa_sub(n, sub_pose_array_topic, 1);   pa_sub.unsubscribe();
    message_filters::Subscriber<people_msgs::People>                 p_sub(n, sub_people_topic, 1);        p_sub.unsubscribe();

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(pt_sub),
                                                       boost::ref(ps_sub),
                                                       boost::ref(pa_sub),
                                                       boost::ref(p_sub));

    std::string pub_positions_topic, pub_pose_topic, pub_pose_array_topic, pub_people_topic, pub_marker_topic;
    pn.param("filtered_positions", pub_positions_topic, std::string("/people_tracker_filter/positions"));
    pn.param("filtered_pose", pub_pose_topic, std::string("/people_tracker_filter/pose"));
    pn.param("filtered_pose_array", pub_pose_array_topic, std::string("/people_tracker_filter/pose_array"));
    pn.param("filtered_people", pub_people_topic, std::string("/people_tracker_filter/people"));
    pn.param("filtered_marker", pub_marker_topic, std::string("/people_tracker_filter/marker_array"));
    pt_pub = n.advertise<bayes_people_tracker::PeopleTracker>(pub_positions_topic, 10, con_cb, con_cb);
    ps_pub = n.advertise<geometry_msgs::PoseStamped>(pub_pose_topic, 10, con_cb, con_cb);
    pa_pub = n.advertise<geometry_msgs::PoseArray>(pub_pose_array_topic, 10, con_cb, con_cb);
    p_pub = n.advertise<people_msgs::People>(pub_people_topic, 10, con_cb, con_cb);
    ma_pub = n.advertise<visualization_msgs::MarkerArray>(pub_marker_topic, 10, con_cb, con_cb);

    typedef message_filters::sync_policies::ExactTime<bayes_people_tracker::PeopleTracker, geometry_msgs::PoseStamped,
            geometry_msgs::PoseArray, people_msgs::People> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pt_sub, ps_sub, pa_sub, p_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}
