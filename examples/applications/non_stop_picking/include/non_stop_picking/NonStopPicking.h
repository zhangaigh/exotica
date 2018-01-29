#ifndef NON_STOP_PICKING_H
#define NON_STOP_PICKING_H

#include <exotica/Exotica.h>
#include <exotica/Problems/UnconstrainedEndPoseProblem.h>
#include <exotica/Problems/UnconstrainedTimeIndexedProblem.h>
#include <time_indexed_rrt_connect/TimeIndexedRRTConnect.h>
#include <boost/thread.hpp>

using namespace exotica;
struct CTState
{
    Eigen::VectorXd q;
    double t;
};
class NonStopPicking
{
public:
    NonStopPicking();
    ~NonStopPicking();

    bool initialise(const std::string &rrtconnect_filepath, const std::string &endpose_filepath, const std::string &trajectory_filepath, std::string &eef_link);
    bool setConstraint(Trajectory &cons, double start, double end);
    bool solve(const CTState &start, const CTState &goal, Eigen::MatrixXd &solution);
    void solveConstraint(Eigen::VectorXd &q, double t);
    void solveConstraintTrajectory(const Eigen::VectorXd &qs, double ta, double tb, Eigen::MatrixXd &solution);
    void publishTrajectory(const Eigen::MatrixXd &solution);
    unsigned int max_trial_;

    bool has_constraint_;
    Trajectory constraint_;
    double tc_start_;
    double tc_end_;
    std::string eef_link_;
    KDL::Frame default_target_pose_;

    TimeIndexedSamplingProblem_ptr rrtconnect_problem_;
    TimeIndexedRRTConnect_ptr rrtconnect_solver_;
    UnconstrainedEndPoseProblem_ptr endpose_problem_;
    MotionSolver_ptr endpose_solver_;
    UnconstrainedTimeIndexedProblem_ptr trajectory_problem_;
    MotionSolver_ptr trajectory_solver_;
};

typedef std::shared_ptr<NonStopPicking> NonStopPicking_ptr;

class NonStopPickingThreaded
{
public:
    NonStopPickingThreaded();
    ~NonStopPickingThreaded();
    bool initialise(const std::string &rrtconnect_filepath, const std::string &optimization_filepath, unsigned int num_threads = 1);
    void setConstraint(Trajectory &cons, double start, double end);
    bool solve(const CTState &start, const CTState &goal, Eigen::MatrixXd &solution);
    void publishTrajectory(const Eigen::MatrixXd &solution);

private:
    std::vector<NonStopPicking_ptr> NSPs_;
    unsigned int num_threads_;
};
#endif
