#include <iostream>
#include "bsplines/BSplinePose.hpp"

#include <boost/cstdint.hpp>

//#include <sm/eigen/NumericalDiff.hpp>


#include <sm/eigen/gtest.hpp>
#include <sm/eigen/NumericalDiff.hpp>

#include <boost/tuple/tuple.hpp>
#include <sm/kinematics/transformations.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/EulerRodriguez.hpp>
#include <sm/kinematics/EulerAnglesYawPitchRoll.hpp>
#include <sm/kinematics/EulerAnglesZYX.hpp>
#include <stdio.h>

#include <boost/progress.hpp>
#include <Eigen/Dense>

using namespace bsplines;
using namespace sm::kinematics;

const bool benchmark = false;

const int length = 20;
int numSegments = 20; // todo
double lambda = 0.00005; // todo smooth
int order = 7;


void saveTumTxt(const std::string& file, const Eigen::Matrix<double, length, 1>& times, const Eigen::MatrixXd& poses)
{
    EulerAnglesYawPitchRoll rpy;

    printf("\n..............Saving path................\n");
    std::string file_name = file + "_pts" + std::to_string(length);
    std::ofstream of(file_name);
    if (of.is_open())
    {
        of.setf(std::ios::fixed, std::ios::floatfield);
        of.precision(6);
        for (int i = 0; i < (int)times.size(); ++i) {
            Eigen::Vector3d rpy_angle(poses(3, i), poses(4, i), poses(5, i));
            Eigen::Matrix3d r = rpy.parametersToRotationMatrix(rpy_angle);
            Eigen::Quaternion<double> q(r);
            of<< times[i] << " "
              << poses(0, i) << " "
              << poses(1, i) << " "
              << poses(2, i) << " "
              << q.x() << " "
              << q.y() << " "
              << q.z() << " "
              << q.w() << "\n";
        }
        of.close();
    }
}

void saveCurveTumTxt(const std::string& file, const BSplinePose& spline, int interval = 100)
{
    printf("\n..............Saving Curve................\n");

    EulerAnglesYawPitchRoll rpy;
    std::pair<double, double> time_begin_end;
    time_begin_end = spline.timeInterval();
    printf("time begin end: %f --- %f\n", time_begin_end.first, time_begin_end.second);
    double time_interval = (time_begin_end.second - time_begin_end.first) / (double)interval;
    printf("time_interval: %f\n", time_interval);

    std::string file_name = file + "_segments_" +  std::to_string(numSegments) +
            "_order_" +  std::to_string(order) + "_lambda" +  std::to_string(lambda);
    std::ofstream of(file_name);
    if (of.is_open())
    {
        of.setf(std::ios::fixed, std::ios::floatfield);
        of.precision(6);
        for (int i = 0; i < interval + 1; ++i) {
            double time_now = time_begin_end.first + i * time_interval;
            Eigen::VectorXd pose = spline.eval(time_now);
            Eigen::Vector3d rpy_angle(pose(3, i), pose(4, i), pose(5, i));
            Eigen::Matrix3d r = rpy.parametersToRotationMatrix(rpy_angle);
            Eigen::Quaternion<double> q(r);
            of<< time_now << " "
                << pose(0, i) << " "
                << pose(1, i) << " "
                << pose(2, i) << " "
              << q.x() << " "
              << q.y() << " "
              << q.z() << " "
              << q.w() << "\n";
        }
        of.close();
    }
}

int main() {
    srand(time(nullptr));

    // random positive times
    Eigen::Matrix<double, length, 1> times = Eigen::VectorXd::Random(length) + Eigen::VectorXd::Ones(length);
    // sorted
    std::sort(&times.coeffRef(0), &times.coeffRef(0)+times.size());

    times[length-1] = ceil(times[length-1]);

    double l = 1.0;
    Eigen::MatrixXd poses = Eigen::MatrixXd::Random(6,length);
    for (int i = 0; i < length; ++i) {
        poses(0, i) = i * l;
//                poses(1, i) = rand() % 1;
//                poses(2, i) = rand() % 1;
        poses(1, i) *= 2.0;
        poses(1, i) *= 2.0;
    }

    saveTumTxt("/tmp/control_points", times, poses);

    boost::shared_ptr<RotationalKinematics> rvs;
    rvs.reset(new RotationVector());

    {
        order = 3;
        // Create a two segment spline.
        BSplinePose bs_sparse(order, rvs);
        BSplinePose bs_dense(order, rvs);
        {
            boost::progress_timer timer;
            std::cout << "Dense:" << std::endl;
            bs_dense.initPoseSpline3(times, poses, numSegments, lambda);
            saveCurveTumTxt("/tmp/curve_dense", bs_dense);
        }
        {
            boost::progress_timer timer;
            std::cout << "Sparse:" << std::endl;
            bs_sparse.initPoseSplineSparse(times, poses, numSegments, lambda);
            saveCurveTumTxt("/tmp/curve_sparse", bs_dense);
        }
    }

    {
        order = 5;
        // Create a two segment spline.
        BSplinePose bs_sparse(order, rvs);
        BSplinePose bs_dense(order, rvs);
        {
            boost::progress_timer timer;
            std::cout << "Dense:" << std::endl;
            bs_dense.initPoseSpline3(times, poses, numSegments, lambda);
            saveCurveTumTxt("/tmp/curve_dense", bs_dense);
        }
        {
            boost::progress_timer timer;
            std::cout << "Sparse:" << std::endl;
            bs_sparse.initPoseSplineSparse(times, poses, numSegments, lambda);
            saveCurveTumTxt("/tmp/curve_sparse", bs_dense);
        }
    }

    {
        order = 2;
        // Create a two segment spline.
        BSplinePose bs_sparse(order, rvs);
        BSplinePose bs_dense(order, rvs);
        {
            boost::progress_timer timer;
            std::cout << "Dense:" << std::endl;
            bs_dense.initPoseSpline3(times, poses, numSegments, lambda);
            saveCurveTumTxt("/tmp/curve_dense", bs_dense);
        }
        {
            boost::progress_timer timer;
            std::cout << "Sparse:" << std::endl;
            bs_sparse.initPoseSplineSparse(times, poses, numSegments, lambda);
            saveCurveTumTxt("/tmp/curve_sparse", bs_dense);
        }
    }

    return 0;
}
