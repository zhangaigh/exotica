#include <non_stop_picking/NonStopPicking.h>

using namespace exotica;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "NonStopPicking");
    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));
    NonStopPicking nsp;

    std::string rrt_connect_file, optimization_file, constraint_file;
    Server::getParam("RRTConnectConfigFile", rrt_connect_file);
    Server::getParam("OptimizationConfigFile", optimization_file);
    nsp.initialise(rrt_connect_file, optimization_file);

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