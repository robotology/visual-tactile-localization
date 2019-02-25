#include <Logger.h>

#include <Eigen/Dense>

#include <yarp/eigen/Eigen.h>

#include <iostream>

using namespace Eigen;
using namespace yarp::eigen;

ObjectTrackingLogger::ObjectTrackingLogger(const std::string port_prefix, const double period) :
    port_prefix_(port_prefix),
    period_(period),
    run_(false),
    quit_(false)
{ }


ObjectTrackingLogger::~ObjectTrackingLogger()
{ }


bool ObjectTrackingLogger::run()
{
    enable_log(".", "data");

    mutex_.lock();

    run_ = true;

    mutex_.unlock();

    return true;
}


bool ObjectTrackingLogger::stop()
{
    disable_log();

    mutex_.lock();

    run_ = false;

    mutex_.unlock();

    return true;
}


bool ObjectTrackingLogger::quit()
{
    disable_log();

    mutex_.lock();

    quit_ = true;

    stopModule();

    mutex_.unlock();

    return true;
}


bool ObjectTrackingLogger::configure(yarp::os::ResourceFinder& rf)
{
    bool ports_ok = true;

    ports_ok &= port_estimate_in_.open("/" + port_prefix_ + "/estimate:i");

    ports_ok &= port_ground_truth_0_in_.open("/" + port_prefix_ + "/ground-truth-0:i");

    ports_ok &= port_ground_truth_1_in_.open("/" + port_prefix_ + "/ground-truth-1:i");

    ports_ok &= port_execution_time_in_.open("/" + port_prefix_ + "/execution-time:i");

    ports_ok &= port_rpc_command_.open("/" + port_prefix_ + "/cmd:i");

    ports_ok &= this->yarp().attachAsServer(port_rpc_command_);

    return ports_ok;
}


double ObjectTrackingLogger::getPeriod()
{
    return period_;
}


bool ObjectTrackingLogger::updateModule()
{
    bool run_local;

    bool quit_local;

    mutex_.lock();

    run_local = run_;
    quit_local = quit_;

    mutex_.unlock();

    if (!quit_local)
    {
        if (run_local)
        {
            yarp::sig::Vector* estimate = port_estimate_in_.read(true);
            yarp::sig::Vector* ground_truth_0 = port_ground_truth_0_in_.read(true);
            yarp::sig::Vector* ground_truth_1 = port_ground_truth_1_in_.read(true);
            yarp::sig::Vector* execution_time = port_execution_time_in_.read(true);

            VectorXd estimate_eigen = toEigen(*estimate);
            VectorXd ground_truth_0_eigen = toEigen(*ground_truth_0);
            VectorXd ground_truth_1_eigen = toEigen(*ground_truth_1);
            VectorXd execution_time_eigen = toEigen(*execution_time);

            VectorXd estimate_euler(6);
            VectorXd ground_truth_0_euler(6);
            VectorXd ground_truth_1_euler(6);
            estimate_euler.head<3>() = estimate_eigen.head<3>();
            ground_truth_0_euler.head<3>() = ground_truth_0_eigen.head<3>();
            ground_truth_1_euler.head<3>() = ground_truth_1_eigen.head<3>();

            estimate_euler.tail<3>() = axisAngleToEuler210(estimate_eigen.segment(3, 4));
            ground_truth_0_euler.tail<3>() = axisAngleToEuler210(ground_truth_0_eigen.segment(3, 4));
            ground_truth_1_euler.tail<3>() = axisAngleToEuler210(ground_truth_1_eigen.segment(3, 4));

            // std::cout << estimate_eigen.transpose() << std::endl << std::flush;
            // std::cout << ground_truth_0_eigen.transpose() << std::endl << std::flush;
            // std::cout << ground_truth_1_eigen.transpose() << std::endl << std::flush;
            // std::cout << execution_time_eigen.transpose() << std::endl << std::flush;

            logger(estimate_euler.transpose(), ground_truth_0_euler.transpose(), ground_truth_1_euler.transpose(), execution_time_eigen.transpose());
        }

        return true;
    }

    return false;
}


bool ObjectTrackingLogger::close()
{
    port_estimate_in_.close();

    port_ground_truth_0_in_.close();

    port_ground_truth_1_in_.close();

    port_execution_time_in_.close();

    return true;
}

Eigen::Vector3d ObjectTrackingLogger::axisAngleToEuler210(const Eigen::VectorXd& axis_angle)
{
    AngleAxisd angle_axis(axis_angle(3), axis_angle.head<3>());

    return angle_axis.toRotationMatrix().eulerAngles(2, 1, 0);
}
