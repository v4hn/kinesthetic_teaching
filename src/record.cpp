// I started writing this myself but then continued with a chatgpt generated python version.
// Dead code for now...

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/trajectory_monitor.h>

class Recorder {
  planning_scene_monitor::CurrentStateMonitor& sm_;
  planning_scene_monitor::TrajectoryMonitor& tm_;


public:
  Recorder(planning_scene_monitor::CurrentStateMonitor& state_monitor, planning_scene_monitor::TrajectoryMonitor& trajectory_monitor) :
    sm_{ state_monitor },
    tm_{ trajectory_monitor }
  {
    sm_.addUpdateCallback([this](auto& js){ jsCB(js); });
  }

  void jsCB(const sensor_msgs::JointStateConstPtr& js){

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_demonstration");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner{ 2 };
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitor psm{"robot_description"};
  psm.startStateMonitor();
  auto& state_monitor{ psm.getStateMonitorNonConst() };
  planning_scene_monitor::TrajectoryMonitor trajectory_monitor{ psm.getStateMonitor() };

  Recorder rec{ *state_monitor, trajectory_monitor };

  ros::waitForShutdown();
  return 0;
}
