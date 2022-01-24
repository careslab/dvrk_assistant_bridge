#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <boost/shared_ptr.hpp>

/* bridge2dvrk
 *std_msgs
 * Bridges between a dvrk assistant commands and the actual hardware
 *
 * Publish:   dvrk and dvrk algorithms
 * Subscribe: /assistant/*
 *
 */
class bridge2dvrk
{
public:
    bridge2dvrk();

private:
    void autocameraRunCallback(const std_msgs::Bool::ConstPtr &msg);
    void autocameraTrackCallack(const std_msgs::String::ConstPtr &msg);
    void autocameraKeepCallback(const std_msgs::String::ConstPtr &msg);
    void autocameraFindToolsCallback(const std_msgs::Empty::ConstPtr &msg);
    void autocameraInnerZoomCallback(const std_msgs::Float32::ConstPtr &msg);
    void autocameraOuterZoomCallback(const std_msgs::Float32::ConstPtr &msg);
    void saveCurrentEcmPositionAs(const std_msgs::String::ConstPtr &msg);

    // ROS objects
    ros::NodeHandle mNodeHandle;
    std::map<std::string, ros::Publisher> mPublisherMap;
    std::map<std::string, ros::Subscriber> mSubscriberMap;
    std::map<std::string, sensor_msgs::JointState> mSavedEcmPositionsMap;
    ros::Subscriber point_sub_;
};

// Constructor
//   Set up publisher and subscriber
bridge2dvrk::bridge2dvrk()
{
    /*****SUBSCRIBER SETUP******/
    //Bool: *True or *False
    mSubscriberMap["run"] = mNodeHandle.subscribe("/assistant/autocamera/run", 100, &bridge2dvrk::autocameraRunCallback, this);
    //String: *Right, *Left, or *Middle
    mSubscriberMap["track"] = mNodeHandle.subscribe("/assistant/autocamera/track", 100, &bridge2dvrk::autocameraTrackCallack, this);
    //String: *Right, *Left, or *Middle
    mSubscriberMap["keep"] = mNodeHandle.subscribe("/assistant/autocamera/keep", 100, &bridge2dvrk::autocameraKeepCallback, this);
    //Empty:
    mSubscriberMap["find_tools"] = mNodeHandle.subscribe("/assistant/autocamera/find_tools", 100, &bridge2dvrk::autocameraFindToolsCallback, this);
    //Float32:
    mSubscriberMap["inner_zoom_value"] = mNodeHandle.subscribe("/assistant/autocamera/inner_zoom_value", 100, &bridge2dvrk::autocameraInnerZoomCallback, this);
    //Float32:
    mSubscriberMap["outer_zoom_value"] = mNodeHandle.subscribe("/assistant/autocamera/outer_zoom_value", 100, &bridge2dvrk::autocameraOuterZoomCallback, this);

    
    /*****PUBLISHER SETUP******/
    //Bool: *True or *False
    mPublisherMap["run"] = mNodeHandle.advertise<std_msgs::Bool>("/autocamera/run", 10, true);
    //String: *Right, *Left, or *Middle
    mPublisherMap["track"] = mNodeHandle.advertise<std_msgs::String>("/autocamera/track", 10, true);
    //String: *Right, *Left, or *Middle
    mPublisherMap["keep"] = mNodeHandle.advertise<std_msgs::String>("/autocamera/keep", 10,  true);
    //Empty:
    mPublisherMap["find_tools"] = mNodeHandle.advertise<std_msgs::Empty>("/autocamera/find_tools", 10, true);
    //Float32:
    mPublisherMap["inner_zoom_value"] = mNodeHandle.advertise<std_msgs::Float32>("/autocamera/inner_zoom_value", 10,true);
    //Float32:
    mPublisherMap["outer_zoom_value"] = mNodeHandle.advertise<std_msgs::Float32>("/autocamera/outer_zoom_value", 10,true);
    
}

// Callback function for the run
void bridge2dvrk::autocameraRunCallback(const std_msgs::Bool::ConstPtr &msg)
{
    mPublisherMap["run"].publish(msg);
}
// Callback function for the track
void bridge2dvrk::autocameraTrackCallack(const std_msgs::String::ConstPtr &msg)
{
    mPublisherMap["track"].publish(msg);
}
// Callback function for the keep
void bridge2dvrk::autocameraKeepCallback(const std_msgs::String::ConstPtr &msg)
{
    mPublisherMap["keep"].publish(msg);
}
// Callback function for the find_tools
void bridge2dvrk::autocameraFindToolsCallback(const std_msgs::Empty::ConstPtr &msg)
{
    mPublisherMap["find_tools"].publish(msg);
}
// Callback function for the inner_zoom_value
void bridge2dvrk::autocameraInnerZoomCallback(const std_msgs::Float32::ConstPtr &msg)
{
    mPublisherMap["inner_zoom_value"].publish(msg);
}
// Callback function for the outer_zoom_value
void bridge2dvrk::autocameraOuterZoomCallback(const std_msgs::Float32::ConstPtr &msg)
{
    mPublisherMap["outer_zoom_value"].publish(msg);
}
void bridge2dvrk::saveCurrentEcmPositionAs(const std_msgs::String::ConstPtr &msg)
{

    boost::shared_ptr<sensor_msgs::JointState const> sharedPtr;
    sensor_msgs::JointState js;

    //Get joint angles from the simulation
    sharedPtr  = ros::topic::waitForMessage<sensor_msgs::JointState>("/dvrk_ecm/joint_states", ros::Duration(1));
    if (sharedPtr == NULL)
        ROS_INFO("No Current Joint angle messages received");
    else
        js = *sharedPtr;
        mSavedEcmPositionsMap[msg->data.c_str()] = js;


//     """Move the arm to the end vector by passing the trajectory generator.

//         :param end_joint: the list of joints in which you should conclude movement
//         :returns: true if you had succesfully move
//         :rtype: Bool"""
// #         rospy.loginfo(rospy.get_caller_id() + ' -> starting move joint direct')
//         if (self.__check_input_type(end_joint, [list,float])):
//             if not self.__dvrk_set_state('DVRK_POSITION_JOINT'):
//                 return False
//             # go to that position directly
//             joint_state = JointState()
//             joint_state.position[:] = end_joint
//             self.set_position_joint_publisher.publish(joint_state)
// #             rospy.loginfo(rospy.get_caller_id() + ' <- completing move joint direct')
//             return True
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Running dvrk assistant bridge!!");
    ros::init(argc, argv, "assistant_bridge");
    bridge2dvrk b;

    ros::spin();
}
