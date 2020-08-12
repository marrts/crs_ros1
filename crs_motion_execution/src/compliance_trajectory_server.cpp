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


static const std::string DESIRED_WRENCH_TOPIC = "/pos_compliance_controller/target_wrench";
static const std::string POSITION_COMPLIANCE_TOPIC = "/pos_compliance_controller/target_frame";
static const std::string WRENCH_TOPIC = "/wrench";
static const std::string SANDER_WRENCH_TOPIC = "/sander_wrench";
static const std::string SANDER_SPEED_TOPIC = "/sander_speed";
static const std::string ZERO_FTSENSOR_SERVICE = "/ur_hardware_interface/zero_ftsensor";

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
    target_wrench_pub_ = nh.advertise<geometry_msgs::WrenchStamped>(DESIRED_WRENCH_TOPIC, 10);
    target_frame_pub_ = nh.advertise<geometry_msgs::PoseStamped>(POSITION_COMPLIANCE_TOPIC, 10);
    curr_sander_wrench_ = nh.advertise<geometry_msgs::WrenchStamped>(SANDER_WRENCH_TOPIC, 10);
    speed_pub_= nh.advertise<std_msgs::Float64>(SANDER_SPEED_TOPIC , 10);
    wrench_sub_ = nh.subscribe<geometry_msgs::WrenchStamped>(WRENCH_TOPIC, 10, &CartAction::wrenchCB, this);

    zero_ftsensor_client_ = nh.serviceClient<std_srvs::Trigger>(ZERO_FTSENSOR_SERVICE);

    as_.start();

  }

protected:

  void executeCB(const cartesian_trajectory_msgs::CartesianComplianceTrajectoryGoalConstPtr &goal)
  {
    std_srvs::Trigger zero_ftsensor_request;
    zero_ftsensor_client_.call(zero_ftsensor_request);

    ros::Rate rate(100);
    bool success = false;
    double targ_speed = goal->speed;
    geometry_msgs::Wrench force = goal->force;
    std::string tcp_frame, parent_frame;
    Eigen::Isometry3d tcp_offset;
    tf::poseMsgToEigen(goal->trajectory.tcp_offset, tcp_offset);
    double curr_force = 0;

    tcp_frame = goal->trajectory.tcp_frame;
//    parent_frame = goal->trajectory.header.frame_id;
    parent_frame = "base_link";
    feedback_.tcp_frame = tcp_frame;
    feedback_.header = goal->trajectory.header;


    double virtual_dist = 0; // Distance required, perpendicular to force, to set the target frame far enough away to achieve desired speed
    double virtual_force_dist = 0; // Offset distance required, perpendicular to force, to set the target frame far enough away to achieve desired force
    double K = 0.05;  // Proportional gain applied to calculate virtual_dist
    double K_vfd = 0.00008; // Proportional gain applied to calculate virtual_force_dist
    double curr_speed = 0;
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

    // Publish target force
    geometry_msgs::WrenchStamped targ_wrench, virtual_targ_wrench;
    targ_wrench.header.frame_id = tcp_frame;
    targ_wrench.header.stamp = ros::Time::now();
    targ_wrench.header.seq = 0;
    targ_wrench.wrench = force;
//    target_wrench_pub_.publish(targ_wrench);
    uint seq = 0;
    geometry_msgs::PoseStamped virtual_targ_pose_prev;
    virtual_targ_pose_prev.pose = goal->trajectory.points[0].pose;

    // Iterate over each point
    for(size_t i = 0; i <goal->trajectory.points.size(); ++i)
    {
      if (i + 1 == goal->trajectory.points.size())
      {
        virtual_dist = 0;
      }
      cartesian_trajectory_msgs::CartesianTrajectoryPoint target_cart_point = goal->trajectory.points[i];

      // Publish target frame and force
      geometry_msgs::PoseStamped targ_pose, virtual_targ_pose;
      targ_pose.header.frame_id = parent_frame;
      targ_pose.header.stamp = ros::Time::now();
      targ_pose.header.seq = seq;
      targ_pose.pose = target_cart_point.pose;
      virtual_targ_pose = targ_pose;
      target_frame_pub_.publish(targ_pose);

      targ_wrench.header.stamp = ros::Time::now();
      targ_wrench.header.seq = seq;
      targ_wrench.wrench = target_cart_point.wrench;
      bool in_tolerance = false;
      bool in_pose_tol = false, in_ori_tol = false, in_force_tol = false;
      while (!in_tolerance)
      {
        double c = 0.95;
        curr_force = target_cart_point.wrench.force.z * (1 - c) + curr_force * c;
        targ_wrench.wrench.force.z = curr_force;
        seq++;
        // Check error
        geometry_msgs::TransformStamped transform_lookup;
        try
        {
          transform_lookup = tf_buffer_.lookupTransform(parent_frame, tcp_frame, ros::Time::now(), ros::Duration(1));
          Eigen::Isometry3d curr_transform, targ_transform, error, virtual_targ_pose_eig;
          tf::transformMsgToEigen(transform_lookup.transform, curr_transform);
          tf::poseMsgToEigen(target_cart_point.pose, targ_transform);
          error.translation() = targ_transform.translation() - curr_transform.translation();
          error.linear() = targ_transform.rotation() * curr_transform.rotation().inverse();

          // Update current speed
          ros::Time curr_lookup_time = transform_lookup.header.stamp;
          ros::Duration time_diff = curr_lookup_time - prev_lookup_time;
          prev_lookup_time = curr_lookup_time;
          Eigen::Vector3d pose_diff = curr_transform.translation() - prev_transform.translation();
          prev_transform = curr_transform;
          double a = 0.98;
          if (time_diff.toSec() < 2)
            curr_speed = curr_speed * a + (pose_diff.norm() / time_diff.toSec()) * (1 - a);

          std_msgs::Float64 speed_msg;
          speed_msg.data = curr_speed;
          speed_pub_.publish(speed_msg);

          Eigen::Vector3d projected_error_dir = error.translation();
          Eigen::Vector3d sub_projected = projected_error_dir.dot(curr_transform.rotation().matrix().col(2)) * curr_transform.rotation().matrix().col(2);

          // Update target frame to adjust speed and force
          Eigen::Isometry3d virtual_targ_pose_prev_eig;
          tf::poseMsgToEigen(virtual_targ_pose_prev.pose, virtual_targ_pose_prev_eig);
          if (speed_control_mode_ && i + 1 != goal->trajectory.points.size() && !in_pose_tol)
          {
            projected_error_dir -= sub_projected;

            // Speed control
            projected_error_dir = projected_error_dir / projected_error_dir.norm();
            double speed_error = targ_speed - curr_speed;
            virtual_dist += speed_error * K;
            Eigen::Vector3d virtual_error = virtual_dist * projected_error_dir;

            // Force control
            if (!in_force_tol)
            {
                double force_error = target_cart_point.wrench.force.z - curr_wrench_.wrench.force.z;
                if (force_error < 0)
                    force_error += goal->path_tolerance.wrench_error.force.z;
                else
                    force_error -= goal->path_tolerance.wrench_error.force.z;
                virtual_force_dist += force_error * K_vfd;
                virtual_error += virtual_force_dist * curr_transform.rotation().matrix().col(2);
            }

            virtual_targ_pose_eig = targ_transform;
            virtual_targ_pose_eig.translation() = curr_transform.translation() + virtual_error;

            double b = 0.0005;
            virtual_targ_pose_eig.translation() = virtual_targ_pose_prev_eig.translation() * b + virtual_targ_pose_eig.translation() * (1 - b);
            tf::poseEigenToMsg(virtual_targ_pose_eig, virtual_targ_pose.pose);
            virtual_targ_pose.header.stamp = ros::Time::now();
            virtual_targ_pose.header.seq = seq;
            virtual_targ_pose_prev = virtual_targ_pose;

            virtual_targ_wrench = targ_wrench;
            virtual_targ_wrench.header.seq = seq;
            target_wrench_pub_.publish(virtual_targ_wrench);
          }
          else if (i + 1 == goal->trajectory.points.size()) // Allow for speed control in z to get off of the part
          {
              projected_error_dir = projected_error_dir / projected_error_dir.norm();
              double speed_error = targ_speed - curr_speed;
              virtual_dist += speed_error * K;
              Eigen::Vector3d virtual_error = virtual_dist * projected_error_dir;

              virtual_targ_pose_eig = targ_transform;
              virtual_targ_pose_eig.translation() = curr_transform.translation() + virtual_error;

              double b = 0.001;
              virtual_targ_pose_eig.translation() = virtual_targ_pose_prev_eig.translation() * b + virtual_targ_pose_eig.translation() * (1 - b);
              tf::poseEigenToMsg(virtual_targ_pose_eig, virtual_targ_pose.pose);
          }
          else
          {
            virtual_targ_pose_eig = targ_transform;
            virtual_targ_pose_eig.translation() = curr_transform.translation() + projected_error_dir;
            double b = 0.1;
            virtual_targ_pose_eig.translation() = virtual_targ_pose_prev_eig.translation() * b + virtual_targ_pose_eig.translation() * (1 - b);
            tf::poseEigenToMsg(virtual_targ_pose_eig, virtual_targ_pose.pose);
            if(in_pose_tol && !in_ori_tol)
            {
              virtual_dist = 0;
              ROS_WARN("In positional tolerance but not in orientation tolerance, MAY SLOW ROBOT MOTION");
            }
            virtual_targ_pose_prev = virtual_targ_pose;
            virtual_targ_wrench = targ_wrench;
            virtual_targ_wrench.header.seq = seq;
          }

          Eigen::AngleAxisd ang_ax(error.rotation());
          geometry_msgs::Vector3 curr_trans_error, curr_rot_error;
          tf::vectorEigenToMsg(curr_transform.rotation().matrix() * error.translation(), curr_trans_error);
          tf::vectorEigenToMsg(ang_ax.axis() * ang_ax.angle(), curr_rot_error);
          // If not within tolerance do nothing & if in tolerance go to next point
          if (i + 1 == goal->trajectory.points.size())
          {
            in_pose_tol = checkTolerance(curr_trans_error, goal->goal_tolerance.position_error);
            in_ori_tol = checkTolerance(curr_rot_error, goal->goal_tolerance.orientation_error);
            if (i < force_waypoint_)
              in_tolerance = in_pose_tol && in_ori_tol;
            else
              in_tolerance = in_pose_tol && in_ori_tol && speed_control_mode_;
          }
          else
          {
            in_pose_tol = checkTolerance(curr_trans_error, goal->path_tolerance.position_error);
            in_ori_tol = checkTolerance(curr_rot_error, goal->path_tolerance.orientation_error);
            if (i < force_waypoint_)
              in_tolerance = in_pose_tol && in_ori_tol;
            else
              in_tolerance = in_pose_tol && in_ori_tol && speed_control_mode_;
          }
          target_frame_pub_.publish(virtual_targ_pose);
          target_wrench_pub_.publish(virtual_targ_wrench);

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
          Eigen::Isometry3d curr_error_eig_transformed = error;
          curr_error_eig_transformed.translation() = curr_transform.rotation().matrix() * error.translation();
          geometry_msgs::Pose curr_error;
          tf::poseEigenToMsg(curr_error_eig_transformed, curr_error);
          feedback_.error.pose = curr_error;
          Eigen::Matrix<double, 6, 1> targ_wrench_eigen, curr_wrench_eigen, error_wrench_eigen;
          tf::wrenchMsgToEigen(curr_wrench_.wrench, curr_wrench_eigen);
          tf::wrenchMsgToEigen(target_cart_point.wrench, targ_wrench_eigen);
          error_wrench_eigen = targ_wrench_eigen - curr_wrench_eigen;
          geometry_msgs::Wrench error_wrench;
          tf::wrenchEigenToMsg(error_wrench_eigen, error_wrench);
          feedback_.error.wrench = error_wrench;
          feedback_.header.stamp = ros::Time::now();
          feedback_.header.seq = seq;

          Eigen::Matrix<double, 6, 1> tolerance_wrench_eigen;
          tf::wrenchMsgToEigen(goal->path_tolerance.wrench_error, tolerance_wrench_eigen);

          if (checkTolerance(error_wrench.force, goal->path_tolerance.wrench_error.force))
          {
              in_force_tol = true;
              virtual_force_dist = 0;
          }
          else
              in_force_tol = false;
          if (!speed_control_mode_ && in_force_tol)
          {
            ROS_WARN("SPEED ON");
            speed_control_mode_ = true;
          }

          as_.publishFeedback(feedback_);

        }
        catch (tf2::TransformException &ex)
        {
          ROS_ERROR("%s",ex.what());
        }
        rate.sleep();
      }
    }
    speed_control_mode_ = false;

    // publish the feedback
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

  void wrenchCB(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
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
    geometry_msgs::Vector3 force_msg, torque_msg;
    force_msg = msg->wrench.force;
    torque_msg = msg->wrench.torque;
    Eigen::Vector3d force_eig, torque_eig;
    Eigen::Matrix<double, 6, 1> wrench_eig;
    tf::wrenchMsgToEigen(msg->wrench, wrench_eig);
    force_eig << wrench_eig(0), wrench_eig(1), wrench_eig(2);
    torque_eig << wrench_eig(3), wrench_eig(4), wrench_eig(5);

    Eigen::Vector3d sander_force_eig = transform_eig.rotation() * force_eig;
    Eigen::Vector3d sander_torque_eig = transform_eig.translation().cross(sander_force_eig) + transform_eig.rotation() * torque_eig;

    Eigen::Matrix<double, 6, 1> wrench_sander_eig;
    wrench_sander_eig << sander_force_eig(0), sander_force_eig(1), sander_force_eig(2), sander_torque_eig(0), sander_torque_eig(1), sander_torque_eig(2);

    tf::wrenchEigenToMsg(wrench_sander_eig * -1, curr_wrench_.wrench);
    curr_wrench_.header = msg->header;
    curr_wrench_.header.frame_id="sander_center_link";
    curr_sander_wrench_.publish(curr_wrench_);
  }

  actionlib::SimpleActionServer<cartesian_trajectory_msgs::CartesianComplianceTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  ros::Publisher target_wrench_pub_, target_frame_pub_, curr_sander_wrench_, speed_pub_;
  ros::Subscriber wrench_sub_;
  ros::ServiceClient  zero_ftsensor_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  cartesian_trajectory_msgs::CartesianComplianceTrajectoryFeedback feedback_;
  cartesian_trajectory_msgs::CartesianComplianceTrajectoryResult result_;
  geometry_msgs::WrenchStamped curr_wrench_;

  bool speed_control_mode_ = false;
  size_t force_waypoint_ = 1;

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
