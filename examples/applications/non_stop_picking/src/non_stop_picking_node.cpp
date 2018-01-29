#include <non_stop_picking/NonStopPicking.h>

using namespace exotica;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "NonStopPicking");
    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));
    NonStopPicking nsp;

    std::string rrt_connect_file, endpose_file, trajectory_file, constraint_file, eef_link;
    Server::getParam("RRTConnectConfigFile", rrt_connect_file);
    Server::getParam("EndPoseConfigFile", endpose_file);
    Server::getParam("TrajectoryOptConfigFile", trajectory_file);
    Server::getParam("EndEffectorLink", eef_link);
    nsp.initialise(rrt_connect_file, endpose_file, trajectory_file, eef_link);

    Server::getParam("ConstraintFile", constraint_file);
    Trajectory cons(loadFile(constraint_file));
    nsp.setConstraint(cons, 4, 6);
    Eigen::MatrixXd solution;
    CTState start, goal;
    nsp.solve(start, goal, solution);
    while (ros::ok())
    {
        nsp.publishTrajectory(solution);
        ros::Duration(1).sleep();
    }
    Setup::Destroy();
}