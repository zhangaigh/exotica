#include <non_stop_picking/NonStopPicking.h>

using namespace exotica;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "NonStopPicking");
    Server::InitRos(std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~")));
    NonStopPicking nsp;

    nsp.initialise(parsePath("{non_stop_picking}/resources/time_indexed_rrt_connect.xml"), parsePath("{non_stop_picking}/resources/ik.xml"));

    Trajectory cons(loadFile(parsePath("{non_stop_picking}/resources/constraint.traj")));
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