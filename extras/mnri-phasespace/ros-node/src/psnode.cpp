#include <iostream>

#include "owl.hpp"
#include "std_msgs/String.h"
#include "phasespace/Camera.h"
#include "phasespace/Cameras.h"
#include "phasespace/Marker.h"
#include "phasespace/Markers.h"
#include "phasespace/Rigid.h"
#include "phasespace/Rigids.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace std;

int main(int argc, char** argv)
{
  // get the owl server address through the command line
  // 'rosrun phasespace phasespace_node 192.168.1.123'
  // 'rosrun phasespace phasespace_node 160.94.220.134'
  string address = argc > 1 ? argv[1] : "localhost";

  //ROS_INFO("After string address");
  ROS_DEBUG("After string address");

  // create the context
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Cameras cameras;
  OWL::Rigids rigids; //D
  ROS_INFO("After OWL context");
  ROS_INFO("After After OWL context");

  // initialize ROS
  ros::init(argc, argv, "phasespace_nodez");
  ros::NodeHandle nh;
  ros::Publisher errorsPub = nh.advertise<std_msgs::String>("phasespace_errors", 1000, true);
  ros::Publisher camerasPub = nh.advertise<phasespace::Cameras>("phasespace_cameras", 1000, true);
  ros::Publisher markersPub = nh.advertise<phasespace::Markers>("phasespace_markers", 1000);
  ros::Publisher rigidsPub = nh.advertise<phasespace::Rigids>("phasespace_rigids", 1000);
  ros::Publisher twistPub= nh.advertise<geometry_msgs::Twist>("phasespace_twist", 1000);

  // simple example
  if(owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0){ 
    ROS_ERROR("Could not connect to the address %s", address.c_str());
    return 0;
  }
  ROS_INFO("Successfully connected to %s!", address.c_str());

  // start streaming
  owl.streaming(1);

  // create tracker -- data taken from a JSON file in the Phasespace Master Client
  uint32_t tracker_id = 0;

  // tracker IDs will start in either 0 or in what the JSON file gives, in our case "8"
  // since we piggy backed onto the "default" profile and 0-7 were already taken
  owl.createTracker(tracker_id, "rigid", "default");

  /*testing with 4 leds*/
  /*
  owl.assignMarker(tracker_id, 0, "0", "pos=-0.0412,0.1478,-0.0473");
  owl.assignMarker(tracker_id, 1, "1", "pos=-0.0734,0.1429,-0.0222");
  owl.assignMarker(tracker_id, 2, "2", "pos=0.0423,0.1551,-0.0232");
  owl.assignMarker(tracker_id, 3, "3", "pos=0.0280,0.1573,-0.0458");
  */

  /*testing with 8 leds - latest - 04/07 */
  owl.assignMarker(tracker_id, 0, "0", "pos=56.1273,-6.87075,31.0018");
  owl.assignMarker(tracker_id, 1, "1", "pos=-53.3547,0.484228,-34.2984");
  owl.assignMarker(tracker_id, 2, "2", "pos=30.0198,0.750542,61.0814");
  owl.assignMarker(tracker_id, 3, "3", "pos=-36.4633,1.47867,-49.2591");
  owl.assignMarker(tracker_id, 4, "4", "pos=-31.3022,2.38061,56.0915");
  owl.assignMarker(tracker_id, 5, "5", "pos=24.4517,-1.57332,-62.1902");
  owl.assignMarker(tracker_id, 6, "6", "pos=-49.026,2.89297,35.787");
  owl.assignMarker(tracker_id, 7, "7", "pos=59.5472,0.457048,-38.214");

  /*testing with 8 leds*/
  /*
  owl.assignMarker(tracker_id, 0, "0", "pos=-64.0019,-6.69047,-27.891");
  owl.assignMarker(tracker_id, 1, "1", "pos=48.8664,8.70612,37.8948");
  owl.assignMarker(tracker_id, 2, "2", "pos=-30.8954,-6.36885,-56.6268");
  owl.assignMarker(tracker_id, 3, "3", "pos=30.3758,1.11869,53.6228");
  owl.assignMarker(tracker_id, 4, "4", "pos=36.0001,2.92543,-47.5952");
  owl.assignMarker(tracker_id, 5, "5", "pos=-21.9857,9.41852,58.5472");
  owl.assignMarker(tracker_id, 6, "6", "pos=51.9572,-0.595778,-31.3071");
  owl.assignMarker(tracker_id, 7, "7", "pos=-50.3165,-8.51366,13.3552");
  */

  /* default with 8 leds
  owl.assignMarker(tracker_id, 8, "8", "pos=28.2066,-0.475287,-61.8231");
  owl.assignMarker(tracker_id, 9, "9", "pos=-31.6467,0.0144243,56.1771");
  owl.assignMarker(tracker_id, 10, "10", "pos=56.2232,-0.294328,-36.1444");
  owl.assignMarker(tracker_id, 11, "11", "pos=-44.3621,-0.867486,41.8709");
  owl.assignMarker(tracker_id, 12, "12", "pos=55.1574,-0.853181,27.8596-");
  owl.assignMarker(tracker_id, 13, "13", "pos=-62.4878,1.86169,-19.5773");
  owl.assignMarker(tracker_id, 14, "14", "pos=38.6674,-1.45447,46.4575");
  owl.assignMarker(tracker_id, 15, "15", "pos=-39.7568,2.06864,-54.8196");
  */

  // Throttling counter
  int n=0;
  int throttleN = 1000;


  static tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  // main loop
  while(ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
    {
      const OWL::Event *event = owl.nextEvent(1000);
      if(!event) continue;

      if(event->type_id() == OWL::Type::ERROR)
        {
          cerr << event->name() << ": " << event->str() << endl;
          std_msgs::String str;
          str.data = event->str();
          errorsPub.publish(str);
        }
      else if(event->type_id() == OWL::Type::CAMERA)
        {
          if(event->name() == string("cameras") && event->get(cameras) > 0)
            {
              phasespace::Cameras out;
              for(OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++)
                {
                  phasespace::Camera cam;
                  //
                  cam.id = c->id;
                  cam.flags = c->flags;
                  cam.x = c->pose[0];
                  cam.y = c->pose[1];
                  cam.z = c->pose[2];
                  cam.qw = c->pose[3];
                  cam.qx = c->pose[4];
                  cam.qy = c->pose[5];
                  cam.qz = c->pose[6];
                  cam.cond = c->cond;
                  //
                  out.cameras.push_back(cam);
                }
              camerasPub.publish(out);
            }
        }
      else if(event->type_id() == OWL::Type::FRAME)
        {
          if(event->find("markers", markers) > 0)
            {
              phasespace::Markers out;
              for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                {
                  phasespace::Marker mout;
                  mout.id = m->id;
                  mout.time = m->time;
                  mout.flags = m->flags;
                  mout.cond = m->cond;
                  mout.x = m->x;
                  mout.y = m->y;
                  mout.z = m->z;
                  out.markers.push_back(mout);
                }
              markersPub.publish(out);
            }
          if(event->find("rigids", rigids) > 0)
          {
            //phasespace::Rigids out;
            //twistmessage tout

            for (OWL::Rigids::iterator r = rigids.begin(); r!= rigids.end(); r++)
            {
              phasespace::Rigids out;
              geometry_msgs::Twist twist_msg;

              //static tf2_ros::TransformBroadcaster broadcaster;
              //geometry_msgs::TransformStamped transformStamped;

              if(r->cond > 0)
              {
                //Transform message
                ros::Time::now();
                ros::Time current_time = ros::Time::now();

                /*
                  cam.qw = c->pose[3];
                  cam.qx = c->pose[4];
                  cam.qy = c->pose[5];
                  cam.qz = c->pose[6];
                */


                transformStamped.header.stamp = current_time;
                transformStamped.header.frame_id = "odom";
                transformStamped.child_frame_id = "robot";
                transformStamped.transform.translation.x = r->pose[0]/1000;
                transformStamped.transform.translation.z = r->pose[1]/1000;
                transformStamped.transform.translation.y = r->pose[2]/1000;
                transformStamped.transform.rotation.x = r->pose[4];
                transformStamped.transform.rotation.z = r->pose[5];
                transformStamped.transform.rotation.y = r->pose[6];
                transformStamped.transform.rotation.w = r->pose[3];

                //transformStamped.transform.translation.x = r->pose[0]/1000;
                //transformStamped.transform.translation.y = r->pose[1]/1000;
                //transformStamped.transform.translation.z = r->pose[2]/1000;
                //transformStamped.transform.rotation.x = r->pose[4];
                //transformStamped.transform.rotation.y = r->pose[5];
                //transformStamped.transform.rotation.z = r->pose[6];
                //transformStamped.transform.rotation.w = r->pose[3];

                broadcaster.sendTransform(transformStamped);

                //Twist message
                twist_msg.linear.x = r->pose[0];
                twist_msg.linear.y = r->pose[1];
                twist_msg.linear.z = r->pose[2];
                //these need to be changed from quaternion to euler
                twist_msg.angular.x = r->pose[3];
                twist_msg.angular.y = r->pose[4];
                twist_msg.angular.z = r->pose[5];

                phasespace::Rigid rout;
                //Phasespace message
                rout.id = r->id;
                rout.time = r->time;
                rout.flags = r->flags;
                rout.cond = r->cond;
                //rout.pose[0] = r->x;
                rout.x =  r->pose[0];
                rout.y = r->pose[1];
                rout.z = r->pose[2];
                rout.qw = r->pose[3];
                rout.qx = r->pose[4];
                rout.qy = r->pose[5];
                rout.qz = r->pose[6];
                //
                out.rigids.push_back(rout);
              }
              rigidsPub.publish(out);
              n++;
              if (n%throttleN == 0)
                twistPub.publish(twist_msg);
            }
          }
        }
    } // while

  owl.done();
  owl.close();
  
  return 0;
}
