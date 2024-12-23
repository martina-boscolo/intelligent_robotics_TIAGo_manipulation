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
        nh_ptr_(nh_ptr),
        moveClient_("move_base", true),
        as_(*nh_ptr, server_name, boost::bind(&FindTags::mainCycle, this, _1), false),
          corridor_done_(false),
          corridor_feedback_sent_(false),
          AlreadyFoundTags(15, false)
    {
        this->as_.start();

        // Initialize the /cmd_vel publisher
        this->cmd_vel_pub_ = this->nh_ptr_->advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 10);

        this->apriltag_sub_ = this->nh_ptr_->subscribe("tag_detections", 1, &FindTags::tagsCallback, this);
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
                    this->AlreadyFoundIds.push_back(tagId);
                    this->AlreadyFoundTags[tagId] = true;
                   
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

                        this->AlreadyFoundValidTags.push_back(pose_in_map_frame);

                        // Publish the feedback
                        this->feedback_.current_detection = pose_in_map_frame;
                        this->feedback_.id = tagId;
                        this->feedback_.progress_status = AlreadyFoundIds.size();
                        this->feedback_.robot_status = "Moving";
                        this->as_.publishFeedback(this->feedback_);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_WARN("Failed to transform pose: %s", ex.what());
                    }
                }
                else
                {
                    if(!this->AlreadyFoundTags[tagId])
                    {
                        this->AlreadyFoundTags[tagId] = true;
                        ROS_INFO("Tag detected: %d", tagId);
                    }     
                }
            }
        }
    }

    void FindTags::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
    {
        static ros::Time corridor_start_time = ros::Time(0);
        ros::Duration corridor_duration(5.0); // Stay in corridor mode for 5 seconds

        latest_laser_msg_ = msg; // Store the latest laser scan
        double corridor_width = 0.0;
        if (corridor_done_)
        {
            // Already in corridor mode, check if duration has passed
            if (ros::Time::now() - corridor_start_time < corridor_duration)
            {
                ROS_INFO("Still navigating in the corridor...");
                navigateInCorridor(msg, cmd_vel_pub_); // Continue navigating
            }
            else
            {
                ROS_INFO("Finished corridor navigation.");
                this->feedback_.id = -1;
                this->feedback_.robot_status = "Finished corridor navigation.";
                this->as_.publishFeedback(this->feedback_);
                corridor_done_ = false; // Exit corridor mode
            }
        }
        else
        {
            // Check if we're entering a corridor
            if (isInCorridor(msg, corridor_width))
            {
                ROS_INFO("Detected corridor. Starting navigation...");
                corridor_done_ = true;
                corridor_start_time = ros::Time::now(); // Record entry time
                navigateInCorridor(msg, cmd_vel_pub_);  // Start navigating
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
        this->inclineHead(M_PI / 4);
        this->extendTorso();
        this->performFullSpin();
        this->inclineHead(-M_PI / 12);

        int waypointIndex = 0;

        this->laser_sub_ = this->nh_ptr_->subscribe("scan", 1, &FindTags::laserCallback, this);

        while (this->AlreadyFoundIds.size() < this->Ids.size() && waypointIndex < WAYPOINT_LIST.size())
        {
            double corridor_width = 0.0;

            if (!corridor_done_ && latest_laser_msg_ && this->isInCorridor(latest_laser_msg_, corridor_width))
            {
                ROS_INFO("Corridor already processed. Skipping...");
            }
            else
            {
                // spin every 3 waypoints
                if (waypointIndex > 0 && waypointIndex % 3 == 0)
                {
                    this->performFullSpin();
                }

                // Use move_base if not in a corridor
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
            this->result_.detected_tags = this->AlreadyFoundValidTags;
        } else{
           this->result_.status_message = "Not all the apriltags have been found!";
           this->result_.ids = this->AlreadyFoundIds;
           this->result_.detected_tags = this->AlreadyFoundValidTags;
        }
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

    void FindTags::inclineHead(float pitch)
    {
        actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);

        ROS_INFO("Waiting for head_controller...");
        ac.waitForServer();

        control_msgs::PointHeadGoal goal;
        goal.target.header.stamp = ros::Time(0);
        goal.target.header.frame_id = "xtion_rgb_optical_frame";
        goal.target.point.x = 0.0;
        goal.target.point.y = tan(pitch); // inclination of 30 degrees
        goal.target.point.z = 1.0;
        goal.pointing_axis.z = 1.0;
        goal.pointing_frame = "xtion_rgb_optical_frame";
        goal.min_duration = ros::Duration(1.0);
        goal.max_velocity = 0.6;

        ROS_INFO("Sending goal to head_controller...");
        ac.sendGoal(goal);

        this->feedback_.id = -1;
        this->feedback_.robot_status = "Inclining head";
        this->as_.publishFeedback(this->feedback_);

        ac.waitForResult();
        ROS_INFO("Robot is looking down");
    }

    void FindTags::performFullSpin() {
        geometry_msgs::Twist twist;
        double angular_velocity = 1.0; // rad/s
        twist.linear.x = 0.0;          // No linear movement
        twist.angular.z = angular_velocity; // Rotate counterclockwise

        double spin_duration = 2 * M_PI / angular_velocity; // Time for a full rotation

        ros::Rate rate(10);           // 10 Hz loop rate
        ros::Time start_time = ros::Time::now();

        ROS_INFO("Starting full spin...");
        this->feedback_.id = -1;
        this->feedback_.robot_status = "Spinning";
        this->as_.publishFeedback(this->feedback_);
        while ((ros::Time::now() - start_time).toSec() < spin_duration) {
            this->cmd_vel_pub_.publish(twist); // Publish twist message
            rate.sleep();
        }

        // Stop the robot after the spin
        twist.angular.z = 0.0;
        this->cmd_vel_pub_.publish(twist);
        ROS_INFO("Spin complete!");
    }

    bool FindTags::isInCorridor(const sensor_msgs::LaserScanConstPtr &msg, double &corridor_width)
    {
        static ros::Time last_feedback_time_ = ros::Time::now(); // To limit feedback frequency
        int n_ranges = msg->ranges.size();
        if (n_ranges == 0)
            return false;

        double left_dist = 0.0, right_dist = 0.0, forward_dist = 0.0;
        int left_count = 0, right_count = 0, forward_count = 0;

        // Compute distances for left, right, and forward directions
        for (int i = 0; i < n_ranges / 2; ++i)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                left_dist += msg->ranges[i];
                left_count++;
            }
        }
        for (int i = n_ranges / 2; i < n_ranges; ++i)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                right_dist += msg->ranges[i];
                right_count++;
            }
        }

        // Analyze forward distances (10% of ranges centered in the forward direction)
        int forward_start = n_ranges * 0.45; // 45% of the range
        int forward_end = n_ranges * 0.55;   // 55% of the range

        for (int i = forward_start; i < forward_end; ++i)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                forward_dist += msg->ranges[i];
                forward_count++;
            }
        }

        double avg_left_dist = (left_count > 0) ? left_dist / left_count : msg->range_max;
        double avg_right_dist = (right_count > 0) ? right_dist / right_count : msg->range_max;
        double avg_forward_dist = (forward_count > 0) ? forward_dist / forward_count : msg->range_max;

        corridor_width = avg_left_dist + avg_right_dist;

        bool parallel_walls = fabs(avg_left_dist - avg_right_dist) < 0.9; // Walls roughly parallel
        bool narrow_width = corridor_width < 3.0;                         // Sufficiently narrow
        bool sufficient_length = avg_forward_dist > 2;                    // At least 2 meters ahead

        if (parallel_walls && narrow_width && sufficient_length)
        {
            // Limit the frequency of "in corridor" feedback
            if ((ros::Time::now() - last_feedback_time_).toSec() > 1.0) // 1-second interval
            {
                ROS_INFO("Robot is in a corridor.");
                this->feedback_.id = -1;
                this->feedback_.robot_status = "In a corridor";
                this->as_.publishFeedback(this->feedback_);
                last_feedback_time_ = ros::Time::now();
            }
            return true;
        }
        else
            return false;
    }

    void FindTags::navigateInCorridor(const sensor_msgs::LaserScan::ConstPtr &msg, ros::Publisher &cmd_vel_pub_)
    {
        double left_dist = 0.0, right_dist = 0.0;
        int n_ranges = msg->ranges.size();

        int left_count = 0, right_count = 0;

        // Compute average distances for left and right sides
        for (int i = 0; i < n_ranges / 2; ++i)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                left_dist += msg->ranges[i];
                left_count++;
            }
        }
        for (int i = n_ranges / 2; i < n_ranges; ++i)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                right_dist += msg->ranges[i];
                right_count++;
            }
        }

        if (left_count > 0)
            left_dist /= left_count;
        if (right_count > 0)
            right_dist /= right_count;

        // Centering logic
        double angular_z = (right_dist - left_dist) * TURN_GAIN;

        // Ensure forward movement
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = (std::min(left_dist, right_dist) > MIN_DISTANCE) ? MAX_SPEED : 0.0;
        cmd_vel_msg.angular.z = angular_z;

        
        // Debug output
        ROS_INFO("Corridor Navigation: Left Dist: %.2f, Right Dist: %.2f, Angular Z: %.2f",
                 left_dist, right_dist, angular_z);

                    if (!corridor_feedback_sent_)
        {
            ROS_INFO("Robot entered the corridor.");
             this->feedback_.id = -1;
            this->feedback_.robot_status = "Navigating in Corridor";
            this->as_.publishFeedback(this->feedback_);
            corridor_feedback_sent_ = true; // Feedback has been sent, set flag
        }
    }
}