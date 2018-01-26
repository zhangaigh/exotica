#include <non_stop_picking/NonStopPicking.h>
using namespace exotica;

NonStopPicking::NonStopPicking() : has_constraint_(false)
{
}

NonStopPicking::~NonStopPicking()
{
}

bool NonStopPicking::initialise(const std::string &rrtconnect_filepath, const std::string &optimization_filepath)
{
    Initializer solver, problem;

    XMLLoader::load(rrtconnect_filepath, solver, problem);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    rrtconnect_problem_ = std::static_pointer_cast<TimeIndexedSamplingProblem>(any_problem);
    rrtconnect_solver_ = std::static_pointer_cast<TimeIndexedRRTConnect>(any_solver);
    rrtconnect_solver_->specifyProblem(rrtconnect_problem_);

    XMLLoader::load(optimization_filepath, solver, problem);
    optimization_problem_ = Setup::createProblem(problem);
    optimization_solver_ = Setup::createSolver(solver);
    optimization_solver_->specifyProblem(optimization_problem_);

    rrtconnect_problem_->getScene()->attachObject("Target", "Table");
    optimization_problem_->getScene()->attachObject("Target", "Table");
}

bool NonStopPicking::setConstraint(Trajectory &cons, double start, double end)
{
    constraint_ = cons;
    tc_start_ = start;
    tc_end_ = end;
    has_constraint_ = true;
}

void NonStopPicking::solveConstraint(Eigen::VectorXd &q, double t)
{
    const UnconstrainedEndPoseProblem_ptr epp = std::static_pointer_cast<UnconstrainedEndPoseProblem>(optimization_problem_);
    Eigen::MatrixXd solution;
    KDL::Frame y = constraint_.getPosition(t - tc_start_);
    epp->setStartState(q);
    epp->Cost.y = {y.p.data[0], y.p.data[1], y.p.data[2], 0.5 * M_PI, 0, -0.5 * M_PI};
    optimization_solver_->Solve(solution);
    q = solution.row(solution.rows() - 1);
}

void NonStopPicking::solveConstraintTrajectory(const Eigen::VectorXd &qs, double ta, double tb, Eigen::MatrixXd &solution)
{
    solution.resize(0, qs.size() + 1);
    double dt = 0.02;
    unsigned int i = 0;
    Eigen::VectorXd q = qs;
    for (double t = ta; t < tb; t += dt)
    {
        solveConstraint(q, t);
        solution.conservativeResize(solution.rows() + 1, solution.cols());
        solution(solution.rows() - 1, 0) = t;
        solution.row(solution.rows() - 1).tail(q.size()) = q;
    }
    solveConstraint(q, tb);
    solution.conservativeResize(solution.rows() + 1, solution.cols());
    solution(solution.rows() - 1, 0) = tb;
    solution.row(solution.rows() - 1).tail(q.size()) = q;
}

void NonStopPicking::publishTrajectory(const Eigen::MatrixXd &solution)
{
    optimization_problem_->getScene()->Update(solution.row(0).tail(solution.cols() - 1), solution(0, 0));
    static KDL::Frame default_pose = optimization_problem_->getScene()->getSolver().FK("Target", KDL::Frame(), "Table", KDL::Frame());
    optimization_problem_->getScene()->attachObjectLocal("Target", "Table", default_pose);
    bool first = true;

    for (int i = 0; i < solution.rows() - 1; i++)
    {
        double t = solution(i, 0);
        Eigen::VectorXd q = solution.row(i).tail(solution.cols() - 1);
        optimization_problem_->getScene()->Update(q, t);

        if (first && t >= tc_end_)
        {
            first = false;
            optimization_problem_->getScene()->attachObject("Target", "lwr_arm_7_link");
        }
        optimization_problem_->getScene()->publishScene();
        optimization_problem_->getScene()->getSolver().publishFrames();
        ros::Duration(solution(i + 1, 0) - t).sleep();
    }
}

bool NonStopPicking::solve(const CTState &start, const CTState &goal, Eigen::MatrixXd &solution)
{
    Eigen::MatrixXd solution_pre, solution_after, solution_constrained;
    Eigen::VectorXd q_goal = rrtconnect_problem_->getGoalState();
    double t_goal = rrtconnect_problem_->getGoalTime();
    Eigen::VectorXd qs = optimization_problem_->applyStartState();
    qs.setRandom();
    solveConstraintTrajectory(qs, tc_start_, tc_end_, solution_constrained);
    Eigen::VectorXd qa = solution_constrained.row(0).tail(q_goal.size()), qb = solution_constrained.row(solution_constrained.rows() - 1).tail(q_goal.size());

    rrtconnect_problem_->setGoalState(qa);
    rrtconnect_problem_->setGoalTime(tc_start_);
    rrtconnect_solver_->Solve(solution_pre);

    rrtconnect_problem_->getScene()->Update(qb, tc_end_);
    rrtconnect_problem_->getScene()->attachObject("Target", "lwr_arm_7_link");

    rrtconnect_problem_->setStartState(qb);
    rrtconnect_problem_->setStartTime(tc_end_);
    rrtconnect_problem_->setGoalState(q_goal);
    rrtconnect_problem_->setGoalTime(t_goal);
    rrtconnect_solver_->Solve(solution_after);

    solution.resize(solution_pre.rows() + solution_constrained.rows() + solution_after.rows(), solution_pre.cols());
    solution << solution_pre,
        solution_constrained,
        solution_after;
    return true;
}

NonStopPickingThreaded::NonStopPickingThreaded()
{
}

NonStopPickingThreaded::~NonStopPickingThreaded()
{
}

bool NonStopPickingThreaded::initialise(const std::string &rrtconnect_filepath, const std::string &optimization_filepath, unsigned int num_threads)
{
    num_threads_ = num_threads;
    NSPs_.resize(num_threads);
    for (int i = 0; i < num_threads; i++)
    {
        NSPs_[i].reset(new NonStopPicking());
        NSPs_[i]->initialise(rrtconnect_filepath, optimization_filepath);
    }
}

void NonStopPickingThreaded::setConstraint(Trajectory &cons, double start, double end)
{
    for (int i = 0; i < num_threads_; i++)
        NSPs_[i]->setConstraint(cons, start, end);
}

bool NonStopPickingThreaded::solve(const CTState &start, const CTState &goal, Eigen::MatrixXd &solution)
{
    std::vector<Eigen::MatrixXd> solutions(num_threads_);
    ros::Time start_time = ros::Time::now();
    for (int i = 0; i < num_threads_; i++)
    {
        boost::thread t(std::bind(&NonStopPicking::solve, NSPs_[i], start, goal, solutions[i]));
        t.join();
    }
    HIGHLIGHT("Duration " << ros::Duration(ros::Time::now() - start_time).toSec());
    for (int i = 0; i < num_threads_; i++)
        HIGHLIGHT("Thread " << i << " length " << solutions[i].size());
}