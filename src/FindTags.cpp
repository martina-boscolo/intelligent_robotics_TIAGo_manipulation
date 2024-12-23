#include "ir2425_group_08/FindTags.h"

namespace ir2425_group_08
{
    // private

    bool FindTags::isNewAndValidTag(apriltag_ros::AprilTagDetection tag)
    {
        if (tag.id.empty())
            return false;

        int tagId = tag.id[0];
        bool isValid = std::find(this->Ids.begin(), this->Ids.end(), tagId) != this->Ids.end();
        bool isNew = std::find(this->AlreadyFoundIds.begin(), this->AlreadyFoundIds.end(), tagId) == this->AlreadyFoundIds.end();

        return isValid && isNew;
    }

    // public ctor

    FindTags::FindTags(NodeHandleShared& nh_ptr,  std::string server_name)
        :
        moveClient_("move_base", true),
        as_(*nh_ptr, server_name, boost::bind(&FindTags::mainCycle, this, _1), false)
    {
        this->as_.start();

        this->sub_ = nh_ptr->subscribe("tag_detections", 1, &FindTags::tagsCallback, this);
        ROS_INFO("Server and tag_detections subscriber started");

        ROS_INFO("Waiting for move_base...");
        moveClient_.waitForServer();
        ROS_INFO("move_base server ready");
    }

    // public callbacks

    void FindTags::tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
    {
        for (const auto &tag : msg->detections)
        {
            if (!tag.id.empty())
            {
                int tagId = tag.id[0];
                if (this->isNewAndValidTag(tag))
                {
                    bool isPresent = std::find(this->AlreadyFoundIds.begin(), this->AlreadyFoundIds.end(), tagId) != this->AlreadyFoundIds.end();
                    
                    if(!isPresent)
                        this->AlreadyFoundIds.push_back(tagId);

                    //LE STAMPE DEVONO ESSERE SISTEMATE
                    if (!isPresent)
                    {
                        
                        ROS_INFO("New valid tag detected: %d", tagId);
                        ROS_INFO_STREAM("Position (respect to camera) - x: " << tag.pose.pose.pose.position.x
                                                                            << ", y: " << tag.pose.pose.pose.position.y
                                                                            << ", z: " << tag.pose.pose.pose.position.z);

                        // Transform the pose from camera frame to map frame
                        geometry_msgs::PoseStamped pose_in_camera_frame;
                        geometry_msgs::PoseStamped pose_in_map_frame;

                        pose_in_camera_frame.header.frame_id = tag.pose.header.frame_id;
                        pose_in_camera_frame.header.stamp = ros::Time(0);  // Copy the frame_id and timestamp
                        pose_in_camera_frame.pose = tag.pose.pose.pose; // Copy the pose

                        try
                        {
                            // Use the TF listener to transform the pose
                            tf_listener_.waitForTransform("map", pose_in_camera_frame.header.frame_id,
                                                        ros::Time(0), ros::Duration(1.0));
                            tf_listener_.transformPose("map", pose_in_camera_frame, pose_in_map_frame);

                            // Print the transformed pose 
                            ROS_INFO_STREAM("Tag ID: " << tagId << " Position (map frame) - x: "
                                                    << pose_in_map_frame.pose.position.x
                                                    << ", y: " << pose_in_map_frame.pose.position.y
                                                    << ", z: " << pose_in_map_frame.pose.position.z);

                            pose_in_map_frame.header.stamp = ros::Time(0);
                            pose_in_map_frame.header.frame_id = "map";

                            this->AlreadyFoundTags.push_back(pose_in_map_frame);

                            // Publish the feedback
                            this->feedback_.current_detection = pose_in_map_frame;
                            this->feedback_.id = tagId;
                            this->feedback_.progress_status = AlreadyFoundIds.size();
                            if (this->moveClient_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
                            {
                                this->feedback_.robot_status = "Moving";
                            }
                            else
                            {
                                this->feedback_.robot_status = "Unknown";
                            }
                            this->as_.publishFeedback(this->feedback_);
                        }
                        catch (tf::TransformException &ex)
                        {
                            ROS_WARN("Failed to transform pose: %s", ex.what());
                        }
                    }
                }
            }
        }
    }

    void FindTags::mainCycle(const ir2425_group_08::FindTagsGoalConstPtr& goal)
    {
        // Deduplicate the IDs in the goal
        this->Ids.clear();
        for (int id : goal->target_ids)
        {
            if (std::find(this->Ids.begin(), this->Ids.end(), id) == this->Ids.end())
            {
                this->Ids.push_back(id);
            }
        }

        // preparing robot
        this->lookDown();
        this->extendTorso();

        int waypointIndex = 0;

        while (this->AlreadyFoundIds.size() < this->Ids.size() && waypointIndex < WAYPOINT_LIST.size())
        {
            move_base_msgs::MoveBaseGoal waypointGoal;
            waypointGoal.target_pose.header.stamp = ros::Time::now();
            waypointGoal.target_pose.header.frame_id = "map";
            waypointGoal.target_pose.pose.position = WAYPOINT_LIST[waypointIndex];
            waypointGoal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Moving to waypoint %d: x=%.2f, y=%.2f",
                     waypointIndex,
                     waypointGoal.target_pose.pose.position.x,
                     waypointGoal.target_pose.pose.position.y);

            this->moveClient_.sendGoal(waypointGoal);

            this->moveClient_.waitForResult();
            ROS_INFO("Waypoint %d reached", waypointIndex);
            waypointIndex++;
        }

        if (waypointIndex >= WAYPOINT_LIST.size())
        {
            ROS_INFO("All waypoints visited");
        }

        ROS_INFO("Publishing result...");
        this->result_.finished = (this->AlreadyFoundIds.size() == this->Ids.size());
        if (this->AlreadyFoundIds.size() == this->Ids.size()){
            this->result_.status_message = "All the apriltags have been found!"; 
            this->result_.ids = this->AlreadyFoundIds;
            this->result_.detected_tags = this->AlreadyFoundTags;
        } else
           this->result_.status_message = "Not all the apriltags have been found!";
           
        this->as_.setSucceeded(this->result_);
    }

    // public methods

    void FindTags::extendTorso()
    {
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("torso_controller/follow_joint_trajectory", true);

        ROS_INFO("Waiting for torso_controller...");
        ac.waitForServer();

        control_msgs::FollowJointTrajectoryGoal goal;
        trajectory_msgs::JointTrajectoryPoint point;

        goal.trajectory.joint_names.push_back("torso_lift_joint");
        point.positions.push_back(0.35);            // Maximum height (adjust based on Tiago specs)
        point.time_from_start = ros::Duration(2.0); // Time to reach the position

        goal.trajectory.points.push_back(point);
        goal.trajectory.header.stamp = ros::Time::now();

        ROS_INFO("Sending goal to torso_controller...");
        ac.sendGoal(goal);

        this->feedback_.id = -1;
        this->feedback_.robot_status = "Extending torso";
        this->as_.publishFeedback(this->feedback_);

        ac.waitForResult();
        ROS_INFO("Torso fully extended");
    }

    void FindTags::lookDown()
    {
        actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);

        ROS_INFO("Waiting for head_controller...");
        ac.waitForServer();

        control_msgs::PointHeadGoal goal;
        goal.target.header.stamp = ros::Time(0);
        goal.target.header.frame_id = "xtion_rgb_optical_frame";
        goal.target.point.x = 0.0;
        goal.target.point.y = tan(M_PI / 6); // inclination of 30 degrees
        goal.target.point.z = 1.0;
        goal.pointing_axis.z = 1.0;
        goal.pointing_frame = "xtion_rgb_optical_frame";
        goal.min_duration = ros::Duration(1.0);
        goal.max_velocity = 0.6;

        ROS_INFO("Sending goal to head_controller...");
        ac.sendGoal(goal);

        this->feedback_.id = -1;
        this->feedback_.robot_status = "Pointing down camera";
        this->as_.publishFeedback(this->feedback_);

        ac.waitForResult();
        ROS_INFO("Robot is looking down");
    }
}