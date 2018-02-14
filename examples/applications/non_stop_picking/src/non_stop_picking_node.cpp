#include <non_stop_picking/NonStopPicking.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
using namespace exotica;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "NonStopPicking");
    //ros::NodeHandle nh;

    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));

    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> traj_ac("/panda/execute_trajectory", true);
    ROS_INFO("Waiting for action server to start.");
    if(!traj_ac.waitForServer(ros::Duration(2)))
    {
        ROS_ERROR("Can not connect to Trajectory Execution Server");
        return 0;
    }

    NonStopPicking nsp;

    std::string rrt_connect_file, endpose_file, trajectory_file, constraint_file, eef_link;
    Server::getParam("RRTConnectConfigFile", rrt_connect_file);
    Server::getParam("EndPoseConfigFile", endpose_file);
    Server::getParam("TrajectoryOptConfigFile", trajectory_file);
    Server::getParam("EndEffectorLink", eef_link);
    nsp.initialise(rrt_connect_file, endpose_file, trajectory_file, eef_link, 8);

    Server::getParam("ConstraintFile", constraint_file);
    Trajectory cons(loadFile(constraint_file));
    nsp.setConstraint(cons, 4, 6);
    Eigen::MatrixXd solution;
    CTState start, goal;
    nsp.solve(start, goal, solution);
    //while (ros::ok())
    // {
    //     nsp.publishTrajectory(solution);
    //     ros::Duration(1).sleep();
    // }

    moveit_msgs::ExecuteTrajectoryGoal traj_goal;
    traj_goal.trajectory.joint_trajectory.header.frame_id = "world_frame";
    traj_goal.trajectory.joint_trajectory.header.seq = 0;
    traj_goal.trajectory.joint_trajectory.header.stamp = ros::Time::now();
    traj_goal.trajectory.joint_trajectory.joint_names = {"panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"};
    traj_goal.trajectory.joint_trajectory.points.resize(solution.rows() - 1);
    int N = traj_goal.trajectory.joint_trajectory.joint_names.size();
    for(int i=0;i<traj_goal.trajectory.joint_trajectory.points.size();i++)
    {

        traj_goal.trajectory.joint_trajectory.points[i].positions.resize(N);
        traj_goal.trajectory.joint_trajectory.points[i].velocities.resize(N);
        
        for(int n =0; n<N;n++)
        {
            traj_goal.trajectory.joint_trajectory.points[i].positions[n] = solution(i+1,n+1);
            traj_goal.trajectory.joint_trajectory.points[i].velocities[n] = 0.0;
        }

        traj_goal.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(solution(i+1,0));
    }
    traj_ac.sendGoal(traj_goal);
    Setup::Destroy();
}