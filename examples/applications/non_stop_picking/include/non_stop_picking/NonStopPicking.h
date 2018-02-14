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

    bool initialise(const std::string &rrtconnect_filepath, const std::string &endpose_filepath, const std::string &trajectory_filepath, const std::string &eef_link, unsigned int num_threads);
    bool setConstraint(Trajectory &cons, double start, double end);
    bool solve(const CTState &start, const CTState &goal, Eigen::MatrixXd &solution);
    void solveConstraint(Eigen::VectorXd &q, double t);
    void solveConstraintTrajectory(const Eigen::VectorXd &qs, double ta, double tb, Eigen::MatrixXd &solution);
    void solveRRTConnectMultiThreads(const Eigen::VectorXd &qa, const Eigen::VectorXd &qb, double ta, double tb, Eigen::MatrixXd &solution);
    void solveRRTConnect(const Eigen::VectorXd qa, const Eigen::VectorXd qb, double ta, double tb, unsigned int tid);
    void publishTrajectory(const Eigen::MatrixXd &solution);
    unsigned int max_trial_;

    bool has_constraint_;
    Trajectory constraint_;
    double tc_start_;
    double tc_end_;
    std::string eef_link_;
    KDL::Frame default_target_pose_;

    std::vector<TimeIndexedSamplingProblem_ptr> rrtconnect_problems_;
    std::vector<TimeIndexedRRTConnect_ptr> rrtconnect_solvers_;
    Eigen::MatrixXd rrtconnect_solution_;
    UnconstrainedEndPoseProblem_ptr endpose_problem_;
    MotionSolver_ptr endpose_solver_;
    UnconstrainedTimeIndexedProblem_ptr trajectory_problem_;
    MotionSolver_ptr trajectory_solver_;

    unsigned int num_threads_;

    double opt_time_;
    double rrtconnect_time_;
};
#endif
