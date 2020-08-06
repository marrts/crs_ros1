#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <ur_msgs/SetIO.h>


static const std::string ZERO_FTSENSOR_SERVICE = "/ur_hardware_interface/zero_ftsensor";
static const std::string SWITCH_CONTROLLER_TOPIC = "/controller_manager/switch_controller";
static const std::string SWITCH_CONTROLLER_SERVICE = "compliance_controller_on";
static const std::string COMPLIANCE_CONTROLLER_NAME = "pos_compliance_controller";
static const std::string JOINT_CONTROLLER_NAME = "scaled_pos_traj_controller";
static const std::string GET_ACTIVE_CONTROLLERS_SERVICE = "/controller_manager/list_controllers";
static const std::string TOGGLE_SANDER_SERVICE = "toggle_sander";
static const std::string UR_IO_SERVICE = "/ur_hardware_interface/set_io";
static const uint8_t SANDER_IO_PIN = 0;

class CRSRobotComms
{
public:
  CRSRobotComms(ros::NodeHandle &nh, std::string name)
  {
    set_controller_service_ = nh.advertiseService(SWITCH_CONTROLLER_SERVICE, &CRSRobotComms::switchControllerCB, this);
    toggle_sander_service_ = nh.advertiseService(TOGGLE_SANDER_SERVICE, &CRSRobotComms::toggleSanderCB, this);

    controller_manager_client_ = nh.serviceClient<controller_manager_msgs::SwitchController>(SWITCH_CONTROLLER_TOPIC);
    zero_ftsensor_client_ = nh.serviceClient<std_srvs::Trigger>(ZERO_FTSENSOR_SERVICE);
    list_controller_client_ = nh.serviceClient<controller_manager_msgs::ListControllers>(GET_ACTIVE_CONTROLLERS_SERVICE);
    set_ur_io_client_ = nh.serviceClient<ur_msgs::SetIO>(UR_IO_SERVICE);
  }

protected:

  bool switchControllerCB(std_srvs::SetBool::Request  &req,
                          std_srvs::SetBool::Response &res)
  {
    std::vector<std::string> start_controllers, stop_controllers;
    controller_manager_msgs::ListControllers list_ctrl_request;
    if(!list_controller_client_.waitForExistence(ros::Duration(3)))
    {
      ROS_ERROR("Controller manager not found");
      res.message = "Controller manager not found";
      res.success = false;
      return false;
    }
    list_controller_client_.call(list_ctrl_request);
    bool joint_active = false, cart_active = false;
    for (auto ctrl : list_ctrl_request.response.controller)
    {
      if(ctrl.name.compare(JOINT_CONTROLLER_NAME) == 0 && ctrl.state.compare("running") == 0)
        joint_active = true;
      else if(ctrl.name.compare(COMPLIANCE_CONTROLLER_NAME) == 0 && ctrl.state.compare("running") == 0)
        cart_active = true;
    }
    if (req.data) // Set to cartesian controller
    {
      if (!joint_active && cart_active)
      {
        ROS_WARN("Cartesian controller already active");
        res.message = "Cartesian controller already active";
        res.success = true;
        return true;
      }
      ROS_WARN("Activating Cartesian control on the robot");
      if (!cart_active)
        start_controllers.push_back(COMPLIANCE_CONTROLLER_NAME);
      if (joint_active)
        stop_controllers.push_back(JOINT_CONTROLLER_NAME);
    }
    else // Set to joint controller
    {
      if (joint_active && !cart_active)
      {
        ROS_WARN("Joint controller already active");
        res.message = "Joint controller already active";
        res.success = true;
        return true;
      }
      ROS_WARN("Activating joint control on the robot");
      if (!joint_active)
        start_controllers.push_back(JOINT_CONTROLLER_NAME);
      if (cart_active)
        stop_controllers.push_back(COMPLIANCE_CONTROLLER_NAME);
    }
    controller_manager_msgs::SwitchController change_ctrl_request;
    change_ctrl_request.request.start_controllers = start_controllers;
    change_ctrl_request.request.stop_controllers = stop_controllers;
    change_ctrl_request.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    change_ctrl_request.request.start_asap = true;
    change_ctrl_request.request.timeout = 1.0;
    if(!controller_manager_client_.waitForExistence(ros::Duration(3)))
    {
      ROS_ERROR("Controller manager not found");
      res.message = "Controller manager not found";
      res.success = false;
      return false;
    }
    controller_manager_client_.call(change_ctrl_request);

    std::cout << "Done" << std::endl;
    if (change_ctrl_request.response.ok)
    {
      res.message = "Controller successfully updated";
      res.success = true;
      return true;
    }
    else
    {
      ROS_ERROR("Controller failed to update");
      res.message = "Controller failed to update";
      res.success = false;
      return false;
    }
  }

  bool toggleSanderCB(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response & res)
  {
      ur_msgs::SetIO ur_io_req;
      if(!set_ur_io_client_.waitForExistence(ros::Duration(3)))
      {
        ROS_ERROR("UR set IO service not found");
        res.message = "UR set IO service not found";
        res.success = false;
        return false;
      }

      ur_io_req.request.fun = ur_msgs::SetIO::Request::FUN_SET_DIGITAL_OUT;
      ur_io_req.request.pin = SANDER_IO_PIN;
      if (req.data) // Turn sander on
      {
          ur_io_req.request.state = ur_msgs::SetIO::Request::STATE_ON;
      }
      else
      {
          ur_io_req.request.state = ur_msgs::SetIO::Request::STATE_OFF;
      }
      set_ur_io_client_.call(ur_io_req);
      if (ur_io_req.response.success)
      {
        res.message = "UR set IO successfully set";
        res.success = true;
        return true;
      }
      else
      {
        ROS_ERROR("UR set IO failed to set");
        res.message = "UR set IO failed to set";
        res.success = false;
        return false;
      }

  }

  ros::ServiceServer set_controller_service_, toggle_sander_service_;
  ros::ServiceClient controller_manager_client_, zero_ftsensor_client_, list_controller_client_, set_ur_io_client_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "crs_robot_comms");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  CRSRobotComms execute_surface_motion(nh, "crs_robot_comms");
  ros::waitForShutdown();

  return 0;
}
