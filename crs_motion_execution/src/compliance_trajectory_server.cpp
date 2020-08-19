#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cartesian_trajectory_msgs/CartesianComplianceTrajectoryAction.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>


static const std::string DESIRED_WRENCH_TOPIC = "/pos_compliance_controller/target_wrench";
static const std::string POSITION_COMPLIANCE_TOPIC = "/pos_compliance_controller/target_frame";
static const std::string WRENCH_TOPIC = "/wrench";
static const std::string SANDER_WRENCH_TOPIC = "/sander_wrench";
static const std::string SANDER_SPEED_TOPIC = "/sander_speed";
static const std::string ZERO_FTSENSOR_SERVICE = "/ur_hardware_interface/zero_ftsensor";
static const std::string TOGGLE_SANDER_SERVICE = "toggle_sander";
static const double MINIMUM_TOUCH_FORCE = 3;

struct processExecutionStates
{
    uint8_t step;
    const uint8_t APPROACH = 0;
    const uint8_t CONTACT_TO_SANDER_ON = 1;
    const uint8_t MOVING = 2;
    const uint8_t RETREAT = 3;
    const uint8_t CORRECTING_ORIENTATION = 10;
};

struct forceControlConfig
{
    // Timing params
    double control_rate = 100;
    double post_zero_pause = 0.5;

    // Other variables
    double targ_approach_speed = 0.05;
    double minimum_touch_force = MINIMUM_TOUCH_FORCE;

    // Speed control params
    double Kp_vel = 0.05;
    double Kp_vel_app = 0.075;

    // Force control params
    double Kp_force = 0.00003;
    double Ki_force = 0.000005;
    double Kd_force = 0.00005;

    // Filter params
    double target_force_filter = 0.95;
    double speed_filter = 0.98;
    double virtual_targ_pose_filter = 0.0005;
    double wrench_filter = 0.9;
};

template<typename T>
void loadParam(YAML::Node& yaml, T& var, std::string name)
{
    try
    {
        if(yaml[name])
            var = yaml[name].as<T>();
        else
            ROS_WARN("No reference to variable %s", name.c_str());
    }
    catch (YAML::InvalidNode& e)
    {
      ROS_ERROR("Invalid node while parsing yaml, %s", e.what());
    }
    catch (YAML::BadConversion& e)
    {
      ROS_ERROR("failed to parse yaml, %s", e.what());
    }
    catch (YAML::KeyNotFound& e)
    {
      ROS_ERROR("Key not found while parsing, %s", e.what());
    }
}

bool loadControllerGains(const std::string& yaml_fp, forceControlConfig& config)
{
    YAML::Node yaml_node = YAML::LoadFile(yaml_fp);

    if (!yaml_node)
    {
      ROS_ERROR("Failed to load into YAML from file %s", yaml_fp.c_str());
      return false;
    }

    loadParam(yaml_node, config.control_rate, "control_rate");
    loadParam(yaml_node, config.post_zero_pause, "post_zero_pause");

    loadParam(yaml_node, config.targ_approach_speed, "targ_approach_speed");
    loadParam(yaml_node, config.minimum_touch_force, "minimum_touch_force");

    loadParam(yaml_node, config.Kp_vel, "Kp_vel");
    loadParam(yaml_node, config.Kp_vel_app, "Kp_vel_app");

    loadParam(yaml_node, config.Kp_force, "Kp_force");
    loadParam(yaml_node, config.Ki_force, "Ki_force");
    loadParam(yaml_node, config.Kd_force, "Kd_force");

    loadParam(yaml_node, config.target_force_filter, "target_force_filter");
    loadParam(yaml_node, config.speed_filter, "speed_filter");
    loadParam(yaml_node, config.virtual_targ_pose_filter, "virtual_targ_pose_filter");
    loadParam(yaml_node, config.wrench_filter, "wrench_filter");

    return true;
}

bool checkTolerance(const geometry_msgs::Vector3& error, const geometry_msgs::Vector3& tolerance)
{
  return (abs(error.x) <= tolerance.x && abs(error.y) <= tolerance.y && abs(error.z) <= tolerance.z);
}

class CartAction
{
public:
  CartAction(ros::NodeHandle &nh, std::string name) :
    as_(nh, "execute_surface_motion", boost::bind(&CartAction::executeCB, this, _1), false)
  , action_name_(name)
  , listener_(tf_buffer_)
  {
    // Initialize ROS communications
    target_wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(DESIRED_WRENCH_TOPIC, 10);
    target_frame_pub_ = nh.advertise<geometry_msgs::PoseStamped>(POSITION_COMPLIANCE_TOPIC, 10);
    curr_sander_wrench_ = nh.advertise<geometry_msgs::WrenchStamped>(SANDER_WRENCH_TOPIC, 10);
    speed_pub_= nh.advertise<std_msgs::Float64>(SANDER_SPEED_TOPIC , 10);
    wrench_sub_ = nh.subscribe<geometry_msgs::WrenchStamped>(WRENCH_TOPIC, 10, &CartAction::wrenchCB, this);

    zero_ftsensor_client_ = nh.serviceClient<std_srvs::Trigger>(ZERO_FTSENSOR_SERVICE);
    turn_on_sander_client_ = nh.serviceClient<std_srvs::SetBool>(TOGGLE_SANDER_SERVICE);

    as_.start();

    // Get path to motion execution config
    std::string pkg_path = ros::package::getPath("crs_motion_execution");
    config_fp_ = pkg_path + "/config/ME_config.yaml";
    loadControllerGains(config_fp_, config_);
  }

protected:

  void executeCB(const cartesian_trajectory_msgs::CartesianComplianceTrajectoryGoalConstPtr &goal)
  {
    // Load potentially updated configuration parameters
    loadControllerGains(config_fp_, config_);

    // Zero force torque sensor on end effector to improve force reading accuracy
    std_srvs::Trigger zero_ftsensor_request;
    zero_ftsensor_client_.call(zero_ftsensor_request);

    // Set the update rate of the controller
    ros::Rate rate(config_.control_rate);

    // Pause the robot to ensure that there is no motion while zeroing force sensor
    ros::Rate sleeper2(1/config_.post_zero_pause);
    sleeper2.sleep();

    // Update variables with the given goal
    double targ_speed = goal->speed; // Target Cartesian speed of the end effector (m/s)
    std::string tcp_frame, parent_frame;
    tcp_frame = goal->trajectory.tcp_frame;
    parent_frame = "base_link";
    feedback_.tcp_frame = tcp_frame;
    feedback_.header = goal->trajectory.header;

    // Initialize variables that will be filtered and/or used for controls
    double curr_targ_force = 0; // Filtered target force value
    double prev_force_error = 0; // Previously applied force (starts at 0 and updated as readings come in)
    double virtual_dist = 0; // Distance required, to set the target frame far enough away to achieve desired speed
    double virtual_force_dist = 0; // Distance required, parallel to force, to set the target frame far enough away to achieve desired force
    double d_force = 0; // Current derivative of force error
    double i_force = 0; // Current integral of force error
    double curr_speed = 0; // Current speed of the tool
    geometry_msgs::PoseStamped virtual_targ_pose_prev; // Virtual target position of the end effector (previous is initialized to starting pose)
    virtual_targ_pose_prev.pose = goal->trajectory.points[0].pose;
    Eigen::Isometry3d virtual_targ_pose_prev_eig; // Virtual target position as an Eigen matrix
    tf::poseMsgToEigen(virtual_targ_pose_prev.pose, virtual_targ_pose_prev_eig);

    // Other various initializations
    uint seq = 0; // Initialize sequence number for publishing feedback
    bool touching_part = false; // Initialize that the robot is not touching the part yet
    bool success = false; // This is if the action call is successful or not, default to false
    // Do transform lookup to initialize prev_transform and prev_lookup_time
    geometry_msgs::TransformStamped transform_lookup;
    Eigen::Isometry3d prev_transform;
    ros::Time prev_lookup_time;
    try
    {
      transform_lookup = tf_buffer_.lookupTransform(parent_frame, tcp_frame, ros::Time::now(), ros::Duration(1));
      tf::transformMsgToEigen(transform_lookup.transform, prev_transform);
      prev_lookup_time = transform_lookup.header.stamp;
      rate.sleep();
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    // Publish target initial target force
    geometry_msgs::WrenchStamped targ_wrench;
    targ_wrench.header.frame_id = tcp_frame;
    targ_wrench.header.stamp = ros::Time::now();
    targ_wrench.header.seq = 0;
    targ_wrench.wrench.force.z = curr_targ_force;
    target_wrench_pub_.publish(targ_wrench);

    // Initialize current state as approaching
    processExecutionStates curr_state;
    curr_state.step = curr_state.APPROACH;

    // Whether or not sander is on, starts off
    bool sander_on = false;

    // Iterate over each point of given trajectory
    for(size_t i = 0; i <goal->trajectory.points.size(); ++i)
    {
      // Determine if this is the last point in the trajectory
      if (curr_state.step != curr_state.RETREAT && i + 1 == goal->trajectory.points.size())
      {
        curr_state.step = curr_state.RETREAT;
        virtual_dist = 0;
      }

      // Store the current true target position
      cartesian_trajectory_msgs::CartesianTrajectoryPoint target_cart_point = goal->trajectory.points[i];

      // Publish target frame and force
      geometry_msgs::PoseStamped targ_pose, virtual_targ_pose;
      targ_pose.header.frame_id = parent_frame;
      targ_pose.header.stamp = ros::Time::now();
      targ_pose.header.seq = seq;
      targ_pose.pose = target_cart_point.pose;
      virtual_targ_pose = targ_pose;
      target_frame_pub_.publish(targ_pose); // Publish initial true target position (this might need to be removed)

      targ_wrench.header.stamp = ros::Time::now();
      targ_wrench.header.seq = seq;
      targ_wrench.wrench = target_cart_point.wrench;

      // Initialize tolerance checks
      bool in_tolerance = false;    // In full Cartesian tolerance
      bool in_pose_tol = false;     // In xyz positional tolerance
      bool in_ori_tol = false;      // In rpy orientational tolerance
      bool in_force_tol = false;    // In force tolerance (currently used to turn on sander)
      while (!in_tolerance)
      {
        // Determine current step in process
        if (curr_state.step == curr_state.APPROACH && curr_wrench_.wrench.force.z >= config_.minimum_touch_force)
            curr_state.step = curr_state.CONTACT_TO_SANDER_ON;
        if (curr_state.step == curr_state.CONTACT_TO_SANDER_ON && in_force_tol)
            curr_state.step = curr_state.MOVING;
        if (curr_state.step == curr_state.MOVING && in_pose_tol && !in_ori_tol)
            curr_state.step = curr_state.CORRECTING_ORIENTATION;

        seq++; // Update sequence count

        if (curr_wrench_.wrench.force.z >= config_.minimum_touch_force)
        {
          // Check if robot is touching the part if tool frame z force exceeds minimum supplied force
          touching_part = true;
        }

        // Filter the target force so there aren't sharp step changes
        curr_targ_force = curr_targ_force * config_.target_force_filter + target_cart_point.wrench.force.z * (1 - config_.target_force_filter);
        targ_wrench.wrench.force.z = curr_targ_force;

        // Do transform lookup to get positional errors, if lookup fails then don't update controller
        geometry_msgs::TransformStamped transform_lookup;
        try
        {
          // Transform lookup to find current transform to tool
          transform_lookup = tf_buffer_.lookupTransform(parent_frame, tcp_frame, ros::Time::now(), ros::Duration(1));
          Eigen::Isometry3d curr_transform, targ_transform, error, virtual_targ_pose_eig;
          tf::transformMsgToEigen(transform_lookup.transform, curr_transform); // Convert transform to Eigen type
          tf::poseMsgToEigen(target_cart_point.pose, targ_transform); // Convert target point to Eigen type

          // Get the transform from the tool to the target point (error)
          error = curr_transform.inverse() * targ_transform;

          // Update current speed
          ros::Time curr_lookup_time = transform_lookup.header.stamp; // Get current time
          ros::Duration time_diff = curr_lookup_time - prev_lookup_time; // Get delta T
          Eigen::Vector3d pose_diff = curr_transform.translation() - prev_transform.translation(); // Get delta X
          prev_lookup_time = curr_lookup_time; // Set current time as previous time for use on next loop
          prev_transform = curr_transform; // Set current transform as previous time for use on next loop
          // Update speed with current speed estimate and a filter
          if (time_diff.toSec() < 2) // Make sure using a recent measurement
            curr_speed = curr_speed * config_.speed_filter + (pose_diff.norm() / time_diff.toSec()) * (1 - config_.speed_filter);

          // Publish speed info for debugging and visualization purposes
          std_msgs::Float64 speed_msg;
          speed_msg.data = curr_speed;
          speed_pub_.publish(speed_msg);

          // Get just translational component of error
          Eigen::Vector3d error_xyz = error.translation();
          // Get the direction of the translational error as a unit vector
          Eigen::Vector3d error_dir = error_xyz / error_xyz.norm();

          // Update force error, derivative of force error, and integral of force error
          double force_error = target_cart_point.wrench.force.z - curr_wrench_.wrench.force.z;
          d_force = (force_error - prev_force_error) * config_.control_rate;
          i_force += (force_error - prev_force_error) / config_.control_rate;
          prev_force_error = force_error; // Set prev force error to current force error

          // Initialize virtual target pose with true target transform
          virtual_targ_pose_eig = targ_transform;
          if (curr_state.step == curr_state.APPROACH)
          {
            // Make sure sander is off while approaching
            std_srvs::SetBool toggle_sander_request;
            toggle_sander_request.request.data = false;
            turn_on_sander_client_.call(toggle_sander_request);

            // Update speed error based on target approach speed
            double speed_error = config_.targ_approach_speed - curr_speed;
            virtual_dist += speed_error * config_.Kp_vel_app; // Proportional control of approach speed
            Eigen::Vector3d virtual_error = error.translation(); // Initialize virtual error with true error
            virtual_error.z() = virtual_dist; // Override z error with approach virtual error to speed up approach

            // Override translational component of error with virtual error
            virtual_targ_pose_eig.translation() = curr_transform * virtual_error;
          }
          else if (curr_state.step == curr_state.MOVING)
          {
            // Speed control, adjust magnitude of virtual distance and apply it in direction of error
            double speed_error = targ_speed - curr_speed;
            virtual_dist += speed_error * config_.Kp_vel;
            Eigen::Vector3d virtual_error = virtual_dist * error_dir;

            // Force control, apply PID control in direction of force
            virtual_force_dist += force_error * config_.Kp_force + d_force * config_.Kd_force + i_force * config_.Ki_force;
            virtual_error.z() += virtual_force_dist;

            // Override translational component of error with virtual error
            virtual_targ_pose_eig.translation() = curr_transform * virtual_error;
          }
          else if (curr_state.step == curr_state.RETREAT)
          {
              // Make sure sander is off while retreating
              std_srvs::SetBool toggle_sander_request;
              toggle_sander_request.request.data = false;
              turn_on_sander_client_.call(toggle_sander_request);

              // Do speed control towards target retreat point
              double speed_error = targ_speed - curr_speed;
              virtual_dist += speed_error * config_.Kp_vel;
              Eigen::Vector3d virtual_error = virtual_dist * error_dir;

              // Override translational component of error with virtual error
              virtual_targ_pose_eig.translation() = curr_transform * virtual_error;
          }
          else if (curr_state.step == curr_state.CORRECTING_ORIENTATION ||  curr_state.step == curr_state.CONTACT_TO_SANDER_ON)
          {
            // Force control, apply control in direction of force
            Eigen::Vector3d virtual_error = error.translation();
            virtual_force_dist += force_error * config_.Kp_force + d_force * config_.Kd_force * 0 + i_force * config_.Ki_force * 0;
            virtual_error.z() += virtual_force_dist;

            // Override translational component of error with virtual error
            virtual_targ_pose_eig.translation() = curr_transform * error_xyz;

            // Reset virtual distance to 0 to prevent rapid acceleration once in orientational tolerance
            if(curr_state.step == curr_state.CORRECTING_ORIENTATION)
            {
              virtual_dist = 0;
              ROS_WARN("In positional tolerance but not in orientation tolerance, MAY SLOW ROBOT MOTION");
            }
          }
          // Update virtual target pose by filtering with previous target pose
          virtual_targ_pose_eig.translation() = virtual_targ_pose_prev_eig.translation() * config_.virtual_targ_pose_filter +
                  virtual_targ_pose_eig.translation() * (1 - config_.virtual_targ_pose_filter);
          tf::poseEigenToMsg(virtual_targ_pose_eig, virtual_targ_pose.pose);
          virtual_targ_pose.header.stamp = ros::Time::now();
          virtual_targ_pose.header.seq = seq;

          // Update previous target pose as current target to be used in filter of next loop
          virtual_targ_pose_prev = virtual_targ_pose;
          tf::poseMsgToEigen(virtual_targ_pose_prev.pose, virtual_targ_pose_prev_eig);

          // Calculate xyz rotational error
          Eigen::AngleAxisd ang_ax(error.rotation());
          geometry_msgs::Vector3 curr_trans_error, curr_rot_error;
          tf::vectorEigenToMsg(ang_ax.axis() * ang_ax.angle(), curr_rot_error); // Convert rotational error to geometry msg

          // Convert error to geometry msg
          tf::vectorEigenToMsg(error_xyz, curr_trans_error);

          // If not within tolerance do nothing & if in tolerance go to next point
          if (curr_state.step == curr_state.RETREAT) // If retreating compare to goal tolerance
          {
            in_pose_tol = checkTolerance(curr_trans_error, goal->goal_tolerance.position_error);
            in_ori_tol = checkTolerance(curr_rot_error, goal->goal_tolerance.orientation_error);
            in_tolerance = in_pose_tol && in_ori_tol;
            if (in_tolerance)
            {
                targ_wrench.wrench.force.z = 0;
                virtual_targ_pose_eig = curr_transform;
                tf::poseEigenToMsg(virtual_targ_pose_eig, virtual_targ_pose.pose);
            }
          }
          else
          {
            // Check positional tolerances
            in_pose_tol = checkTolerance(curr_trans_error, goal->path_tolerance.position_error);
            in_ori_tol = checkTolerance(curr_rot_error, goal->path_tolerance.orientation_error);

            // First point must just be in tolerance, subsequent points require sander on
            if (i == 0)
              in_tolerance = in_pose_tol && in_ori_tol;
            else
              in_tolerance = in_pose_tol && in_ori_tol && sander_on;
          }

          // Exit correction orientation state if in orientational tolerance
          if (curr_state.step == curr_state.CORRECTING_ORIENTATION && in_ori_tol)
            curr_state.step = curr_state.MOVING;


          // Calculate and check wrench error
          Eigen::Matrix<double, 6, 1> targ_wrench_eigen, curr_wrench_eigen, error_wrench_eigen;
          tf::wrenchMsgToEigen(curr_wrench_.wrench, curr_wrench_eigen);
          tf::wrenchMsgToEigen(target_cart_point.wrench, targ_wrench_eigen);
          error_wrench_eigen = targ_wrench_eigen - curr_wrench_eigen;
          geometry_msgs::Wrench error_wrench;
          tf::wrenchEigenToMsg(error_wrench_eigen, error_wrench);
          if (checkTolerance(error_wrench.force, goal->path_tolerance.wrench_error.force))
          {
              in_force_tol = true;
          }
          else
              in_force_tol = false;
          if (!sander_on && in_force_tol)
          {
            // Turn sander on
            std_srvs::SetBool toggle_sander_request;
            toggle_sander_request.request.data = true;
            turn_on_sander_client_.call(toggle_sander_request);
            ROS_WARN("Sander ON");
            sander_on = true;
          }

          // Publish target frame and wrench based on previous calculations
          target_frame_pub_.publish(virtual_targ_pose);
          target_wrench_pub_.publish(targ_wrench);

          // Populate feedback msg
          feedback_.desired = target_cart_point;
          feedback_.desired.twist.linear.x = targ_speed;
          geometry_msgs::Pose virtual_target_pose;
          tf::poseEigenToMsg(virtual_targ_pose_eig, virtual_target_pose);
          feedback_.virtual_desired.pose = virtual_target_pose;
          feedback_.virtual_desired.wrench = target_cart_point.wrench;
          feedback_.virtual_desired.twist.linear.x = curr_speed;
          geometry_msgs::Pose curr_pose;
          tf::poseEigenToMsg(curr_transform, curr_pose);
          feedback_.actual.pose = curr_pose;
          feedback_.actual.wrench = curr_wrench_.wrench;
          feedback_.actual.twist.linear.x = curr_speed;
          geometry_msgs::Pose curr_error;
          tf::poseEigenToMsg(error, curr_error);
          feedback_.error.pose = curr_error;
          feedback_.error.wrench = error_wrench;
          feedback_.header.stamp = ros::Time::now();
          feedback_.header.seq = seq;

          // Publish feedback
          as_.publishFeedback(feedback_);

        }
        catch (tf2::TransformException &ex)
        {
          ROS_ERROR("%s",ex.what());
        }
        rate.sleep();
      }
    }

    // publish the feedback one last time
    as_.publishFeedback(feedback_);

    success = true;
    if(success)
    {
      result_.success = success;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

  // Subcription to wrench topic
  void wrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    // Find transform from center of force frame to target reference frame
    geometry_msgs::TransformStamped transform_lookup;
    Eigen::Isometry3d transform_eig;
    try
    {
      transform_lookup = tf_buffer_.lookupTransform("sander_center_link", msg->header.frame_id, ros::Time::now(), ros::Duration(1));
      tf::transformMsgToEigen(transform_lookup.transform, transform_eig);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    // Get forces into Eigen form
    Eigen::Vector3d force_eig, torque_eig;
    Eigen::Matrix<double, 6, 1> wrench_eig;
    tf::wrenchMsgToEigen(msg->wrench, wrench_eig);
    force_eig << wrench_eig(0), wrench_eig(1), wrench_eig(2);
    torque_eig << wrench_eig(3), wrench_eig(4), wrench_eig(5);

    // Transform forces into sander frame
    Eigen::Vector3d sander_force_eig = transform_eig.rotation() * force_eig;
    Eigen::Vector3d sander_torque_eig = transform_eig.translation().cross(sander_force_eig) + transform_eig.rotation() * torque_eig;

    // Put transformed forces back into wrench msg
    Eigen::Matrix<double, 6, 1> wrench_sander_eig;
    wrench_sander_eig << sander_force_eig(0), sander_force_eig(1), sander_force_eig(2), sander_torque_eig(0), sander_torque_eig(1), sander_torque_eig(2);
    tf::wrenchEigenToMsg(wrench_sander_eig * -1, curr_wrench_.wrench);

    // Filter force
    curr_wrench_.wrench.force.z = prev_force_ * config_.wrench_filter + curr_wrench_.wrench.force.z * (1 - config_.wrench_filter);

    // Publish force in sander frame
    curr_wrench_.header = msg->header;
    curr_wrench_.header.frame_id="sander_center_link";
    curr_sander_wrench_.publish(curr_wrench_);

    // Update previous force with curr force
    prev_force_ = curr_wrench_.wrench.force.z;
  }

  actionlib::SimpleActionServer<cartesian_trajectory_msgs::CartesianComplianceTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  ros::Publisher target_wrench_pub_, target_frame_pub_, curr_sander_wrench_, speed_pub_;
  ros::Subscriber wrench_sub_;
  ros::ServiceClient  zero_ftsensor_client_, turn_on_sander_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  cartesian_trajectory_msgs::CartesianComplianceTrajectoryFeedback feedback_;
  cartesian_trajectory_msgs::CartesianComplianceTrajectoryResult result_;
  geometry_msgs::WrenchStamped curr_wrench_;

  std::string config_fp_;
  forceControlConfig config_;

  double prev_force_ = 0;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "execute_surface_motion");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  CartAction execute_surface_motion(nh, "execute_surface_motion");
  ros::waitForShutdown();

  return 0;
}
