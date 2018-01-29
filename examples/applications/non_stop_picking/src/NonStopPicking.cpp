#include <non_stop_picking/NonStopPicking.h>
using namespace exotica;

NonStopPicking::NonStopPicking() : has_constraint_(false)
{
}

NonStopPicking::~NonStopPicking()
{
}

bool NonStopPicking::initialise(const std::string &rrtconnect_filepath, const std::string &endpose_filepath, const std::string &trajectory_filepath, std::string &eef_link)
{
    Initializer solver, problem;

    XMLLoader::load(rrtconnect_filepath, solver, problem);
    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);
    rrtconnect_problem_ = std::static_pointer_cast<TimeIndexedSamplingProblem>(any_problem);
    rrtconnect_solver_ = std::static_pointer_cast<TimeIndexedRRTConnect>(any_solver);
    rrtconnect_solver_->specifyProblem(rrtconnect_problem_);

    XMLLoader::load(endpose_filepath, solver, problem);
    endpose_problem_ = std::static_pointer_cast<UnconstrainedEndPoseProblem>(Setup::createProblem(problem));
    endpose_solver_ = Setup::createSolver(solver);
    endpose_solver_->specifyProblem(endpose_problem_);

    XMLLoader::load(trajectory_filepath, solver, problem);
    trajectory_problem_ = std::static_pointer_cast<UnconstrainedTimeIndexedProblem>(Setup::createProblem(problem));
    trajectory_solver_ = Setup::createSolver(solver);
    trajectory_solver_->specifyProblem(trajectory_problem_);

    rrtconnect_problem_->getScene()->attachObject("Target", "Table");
    eef_link_ = eef_link;

    endpose_problem_->setRho("Position", 1000.0);
    endpose_problem_->setRho("Orientation", 500.0);
    endpose_problem_->setRho("JointLimit", 500.0);

    Eigen::VectorXd qs = rrtconnect_problem_->applyStartState();
    rrtconnect_problem_->getScene()->Update(qs, 0);
    default_target_pose_ = rrtconnect_problem_->getScene()->getSolver().FK("Target", KDL::Frame(), "Table", KDL::Frame());

    rrtconnect_problem_->getScene()->getCollisionScene()->setWorldLinkPadding(0.01);
    rrtconnect_problem_->getScene()->updateCollisionObjects();

    for (int i = 0; i < trajectory_problem_->getT(); i++) {
        trajectory_problem_->setRho("Position", 10000.0, i);
        trajectory_problem_->setRho("Orientation", 5000.0, i);
        trajectory_problem_->setRho("JointLimit", 5000.0, i);
    }
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
    Eigen::MatrixXd solution;
    KDL::Frame y = constraint_.getPosition(t - tc_start_);
    endpose_problem_->setStartState(q);
    Eigen::Vector3d target_pos = Eigen::Vector3d(y.p.data[0], y.p.data[1], y.p.data[2]);
    endpose_problem_->setGoal("Position", target_pos);
    endpose_solver_->Solve(solution);
    q = solution.row(solution.rows() - 1);
}

void NonStopPicking::solveConstraintTrajectory(const Eigen::VectorXd &qs, double ta, double tb, Eigen::MatrixXd &solution)
{
    Eigen::MatrixXd tmp_solution;
    trajectory_problem_->setStartState(qs);
    // Initial Trajectory
    std::vector<Eigen::VectorXd> initial_guess;
    initial_guess.assign(trajectory_problem_->getT(), qs);
    trajectory_problem_->setInitialTrajectory(initial_guess);
    
    trajectory_solver_->Solve(tmp_solution);
    solution.resize(tmp_solution.rows(),tmp_solution.cols()+1);
    double dt = (tb-ta)/(tmp_solution.rows()-1.0);
    unsigned int i = 0;
    for (double t = ta; t < tb; t += dt)
    {
        solution(i, 0) = t;
        solution.row(i).tail(tmp_solution.cols()) = tmp_solution.row(i);
        i++;
    }
}

void NonStopPicking::publishTrajectory(const Eigen::MatrixXd &solution)
{
    rrtconnect_problem_->getScene()->attachObjectLocal("Target", "Table", default_target_pose_);
    bool first = true;

    for (int i = 0; i < solution.rows() - 1; i++)
    {
        double t = solution(i, 0);
        Eigen::VectorXd q = solution.row(i).tail(solution.cols() - 1);
        rrtconnect_problem_->getScene()->Update(q, t);

        if (first && t >= tc_end_)
        {
            first = false;
            rrtconnect_problem_->getScene()->attachObject("Target", eef_link_);
        }
        rrtconnect_problem_->getScene()->publishScene();
        rrtconnect_problem_->getScene()->getSolver().publishFrames();
        ros::Duration(solution(i + 1, 0) - t).sleep();
    }
}

bool NonStopPicking::solve(const CTState &start, const CTState &goal, Eigen::MatrixXd &solution)
{
    Eigen::MatrixXd solution_pre, solution_after, solution_constrained;
    Eigen::VectorXd q_goal = rrtconnect_problem_->getGoalState();
    double t_goal = rrtconnect_problem_->getGoalTime();
    Eigen::VectorXd qs = endpose_problem_->applyStartState();
    qs.setRandom();
    solveConstraint(qs, tc_start_);
    solveConstraintTrajectory(qs, tc_start_, tc_end_, solution_constrained);
    Eigen::VectorXd qa = solution_constrained.row(0).tail(q_goal.size()), qb = solution_constrained.row(solution_constrained.rows() - 1).tail(q_goal.size());

    rrtconnect_problem_->setGoalState(qa);
    rrtconnect_problem_->setGoalTime(tc_start_);
    rrtconnect_solver_->Solve(solution_pre);

    rrtconnect_problem_->getScene()->Update(qb, tc_end_);
    rrtconnect_problem_->getScene()->attachObject("Target", eef_link_);

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
        // NSPs_[i]->initialise(rrtconnect_filepath, optimization_filepath);
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