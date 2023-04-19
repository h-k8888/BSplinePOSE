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

using namespace bsplines;
using namespace sm::kinematics;


int main() {
    try {
        boost::shared_ptr<RotationalKinematics> rvs;

        rvs.reset(new RotationVector());

        const bool benchmark = false;

        const int length = benchmark ? 1000 : 100;
        int numSegments = benchmark ? 200 : 20;
        double lambda = 0.1;
        //  int order = 2;
        for(int order = 2; order < 8; order++)
        {
            // Create a two segment spline.
            BSplinePose bs_sparse(order, rvs);
            BSplinePose bs_dense(order, rvs);

            // random positive times
            Eigen::Matrix<double, length, 1> times = Eigen::VectorXd::Random(length) + Eigen::VectorXd::Ones(length);
            // sorted
            std::sort(&times.coeffRef(0), &times.coeffRef(0)+times.size());

            times[length-1] = ceil(times[length-1]);

            //      times = times;

            //  std::cout << "Times:" << std::endl;
            //  std::cout << times << std::endl;

            Eigen::MatrixXd poses = Eigen::MatrixXd::Random(6,length);

//            if(benchmark)
            {
                {
                    boost::progress_timer timer;
                    std::cout << "Dense:" << std::endl;
                    bs_dense.initPoseSpline3(times,poses,numSegments,lambda);
                }
                {
                    boost::progress_timer timer;
                    std::cout << "Sparse:" << std::endl;
                    bs_sparse.initPoseSplineSparse(times,poses,numSegments,lambda);
                }
            }

            // diagonals
            //      for (int i = 0; i < Asparse.rows(); i++)
            //           std::cout << Asparse(i,i) << " : " << Adense(i,i) << std::endl;



            Eigen::MatrixXd m1 = bs_sparse.coefficients();
            Eigen::MatrixXd m2 = bs_dense.coefficients();

//           std::cout << "m1:" << std::endl;
//           std::cout << m1 << std::endl;
//           std::cout << "m2:" << std::endl;
//           std::cout << m2 << std::endl;

//            sm::eigen::assertNear(m1, m2, 1e-8, SM_SOURCE_FILE_POS);

        }
    }
    catch(const std::exception &e) {
//        FAIL() << e.what();
        printf("failure, exit.");
    }




    return 0;
}
