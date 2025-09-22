#include <iostream>
#include <map>
#include <fstream>

#include "ros/ros.h"
#include "owl.hpp"
#include "std_msgs/String.h"
#include "picojson.h"
#include <fstream>

#include <phasespace_msgs/Camera.h>
#include <phasespace_msgs/Cameras.h>
#include <phasespace_msgs/Marker.h>
#include <phasespace_msgs/Markers.h>
#include <phasespace_msgs/Rigids.h>
#include <phasespace_msgs/Rigid.h>
#include <tf2_ros/transform_broadcaster.h>  
#include <geometry_msgs/TransformStamped.h>

/**
 * reads the JSON File specified by the filename, and updates the settings in the OWL context accordingly
 * 
 * returns a map from the rigid tracker id to the frame name
 */
std::map<uint32_t, std::string> loadJSONTrackerFileIntoOWL(OWL::Context &owl, const std::string& trackerFilename) {
    std::map<uint32_t, std::string> idToName;
    picojson::value::array trackersArray;

    // Read file
    {
        std::ifstream trackerFile(trackerFilename);
        if (!trackerFile.is_open()) {
            ROS_ERROR_STREAM("could not open tracker file '" << trackerFilename << "'");
            exit(1);
        }

        std::istream_iterator<char> input(trackerFile);

        picojson::value v;
        std::string err;
        input = picojson::parse(v, input, std::istream_iterator<char>(), &err);
        if (!err.empty())
        {
            ROS_ERROR_STREAM("issue parsing tracker json file: " << err);
            exit(1);
        }

        trackersArray = v.get<picojson::value::object>().at("trackers").get<picojson::value::array>();
    }

    // Load trakcers into owl
    for (auto &tracker : trackersArray)
    {
        picojson::value::object &trackerObject = tracker.get<picojson::value::object>();
        
        uint32_t trackerId = trackerObject.at("id").get<double>();
        auto& trackerName = trackerObject.at("name").get<std::string>();
        auto& trackerType = trackerObject.at("type").get<std::string>();
        auto& trackerOptions = trackerObject.at("options").get<std::string>();

        idToName[trackerId] = trackerName;

        owl.createTracker(trackerId, trackerType, trackerName, trackerOptions);

        for (auto &marker : trackerObject.at("markers").get<picojson::value::array>())
        {
            picojson::value::object &markerObject = marker.get<picojson::value::object>();
            owl.assignMarker(trackerId, markerObject.at("id").get<double>(), markerObject.at("name").get<std::string>(), markerObject.at("options").get<std::string>());
        }
    }
    return idToName;
}

int main(int argc, char **argv)
{

    // init ros stuff
    ros::init(argc, argv, "phasespace_rigid_tracker");
    ros::NodeHandle nh;

    ros::Publisher markersPub = nh.advertise<phasespace_msgs::Markers>("markers", 1000);
    ros::Publisher rigidsPub = nh.advertise<phasespace_msgs::Rigids>("rigids", 1000);
    tf2_ros::TransformBroadcaster tf2Pub;

    // get the owl server address through the command line
    // 'rosrun phasespace phasespace_node 192.168.1.123'
    std::string address;
    std::string trackerFilename;

    {
        ros::NodeHandle privateNh("~");
        privateNh.getParam("phasespace_url", address);
        privateNh.getParam("tracker_filename", trackerFilename);
    }

    // connect to phasespace
    ROS_INFO("connecting to %s . . .", address.c_str());
    OWL::Context owl;
    if (owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0)
    {
        ROS_ERROR("Could not connect to the address %s", address.c_str());
        return 1;
    }
    ROS_INFO("successfully connected to %s!", address.c_str());
    ROS_INFO("remember to start the rosbag command to record topic data to bag!");
    uint64_t startedOn = ros::Time::now().toNSec();

    // connect to phasespace
    owl.streaming(1);

    // create tracker -- data taken from a JSON file created in the master client
    auto trackerIdToName = loadJSONTrackerFileIntoOWL(owl, trackerFilename);
    ROS_INFO("successfully added trackers!");

    // space to hold stuff
    OWL::Markers markers;
    OWL::Rigids rigids;

    int marker_seq = 0;
    int rigid_seq = 0;


    while (ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
    {
        const OWL::Event *event = owl.nextEvent(1000);
        if (!event)
        {
            continue;
        }

        if (event->type_id() == OWL::Type::ERROR)
        {
            ROS_ERROR_STREAM(event->name() << ": " << event->str());
        }
        else if (event->type_id() == OWL::Type::FRAME)
        {
            if (event->find("markers", markers) > 0)
            {
                phasespace_msgs::Markers out;
                for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                {
                    phasespace_msgs::Marker mout;

                    mout.header.stamp.fromNSec(mout.time);
                    mout.header.seq = marker_seq;
                    marker_seq += 1;
                    mout.header.frame_id = "/phasespace";

                    mout.flags = m->flags;
                    mout.cond = m->cond;
                    mout.x = m->x;
                    mout.y = m->y;
                    mout.z = m->z;
                    out.markers.push_back(mout);

                }
                markersPub.publish(out);
            }
            if (event->find("rigids", rigids) > 0)
            {
                phasespace_msgs::Rigids out;
                for (OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++)
                {
                    phasespace_msgs::Rigid rOut;
                    rOut.id = r->id;
                    rOut.flags = r->flags;

                    // phasespace time is microseconds since we started tracking 
                    // (i.e microseconds since the node was turned on)
                    // we convert it to unix nanos
                    rOut.time = (r->time * 1000) + startedOn;

                    rOut.header.stamp.fromNSec(rOut.time);
                    rOut.header.seq = rigid_seq;
                    rigid_seq += 1;

                    rOut.header.frame_id = "/phasespace";

                    rOut.cond = r->cond;
                    rOut.x = r->pose[0];
                    rOut.y = r->pose[1];
                    rOut.z = r->pose[2];
                    rOut.qw = r->pose[3];
                    rOut.qx = r->pose[4];
                    rOut.qy = r->pose[5];
                    rOut.qz = r->pose[6];
                    out.rigids.push_back(rOut);

                    // publish transform to tf
                    if (trackerIdToName[rOut.id] != "") {
                        geometry_msgs::TransformStamped rigidTransform;
                        rigidTransform.header.stamp = ros::Time::now(); // (0, rOut.time);
                        rigidTransform.header.frame_id = "phasespace";
                        rigidTransform.child_frame_id = trackerIdToName[rOut.id];
                        rigidTransform.transform.translation.x = rOut.x / 1000;
                        rigidTransform.transform.translation.y = rOut.y / 1000;
                        rigidTransform.transform.translation.z = rOut.z / 1000;
                        rigidTransform.transform.rotation.x = rOut.qx;
                        rigidTransform.transform.rotation.y = rOut.qy;
                        rigidTransform.transform.rotation.z = rOut.qz;
                        rigidTransform.transform.rotation.w = rOut.qw;
                        tf2Pub.sendTransform(rigidTransform);
                    }
                }

                rigidsPub.publish(out);
            }
        }

        ros::spinOnce();
    }

    owl.done();
    owl.close();

    return 0;
}