#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include "std_msgs/Float32MultiArray.h"
#include "rotors_gazebo/Num.h"
#include <opencv2/core/core.hpp>

//4x1 eigenvector + corresponding eigenvalue (only for doubles) strucure
struct EigenStuff {
    double eigen_value_s;
    Eigen::Matrix<std::complex<double>,4,1> eigen_vector_s;
};

bool sortingFunction (EigenStuff e1, EigenStuff e2) {
    return (e1.eigen_value_s<e2.eigen_value_s);
}

int solveLP_returnStatus() {

}

class OdometryWrapperWithAlgorithm {
public:
    void OdometryFirefly1MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void OdometryFirefly2MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void OdometryFirefly3MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void OdometryFirefly4MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    void CalculateMatrices(); //Hard coded 4x4
    void CalculateFiedlerLaplacianDerivative(); //Hard coded 4x4
    void CalculateLaplacianCoordinateDerivatives(); //12 matrices which indicate diretions for robots movements
    void CalculateTraceValues();
    void CalculateEucilidanDistancesMatrix();
    void CalculateAKappaMatrix();
    void CalculateAKappaGradientMatrix();
    void CalculateLP();

    void AlgorithmMainMethod();

    void PrintWeightedLaplacian(int laplacian_dimension);
    void PrintConnectivityMatrix(int connectivity_matrix_dimension);
    void PrintDFiedlerDL(int dF_dL_dimension);
    void Print44DoubleMatrix(Eigen::Matrix<double,4,4> m);


    double* GetTraceValues();
    double* GetLPsolutions();
    double GetSigmoid(double x);
    double GetSigmoidDerivative(double x);
    double EuclidianDistanceFromTheOrigin(Eigen::Vector3d v); // also calculates and 'gets' ...
    Eigen::Matrix<double,1,12> Get_axy_Gradient(int x, int y);
    Eigen::Matrix<double,1,12> GetCabGradientAndFillCab_(int x, int y);
    Eigen::Matrix<double,48,48> GetKroneckerProduct(Eigen::Matrix<double,4,4> m1, Eigen::Matrix<double,12,12> m2); // Hardcoded

    //void PrintLaplaciansOverCoordinatesMatrices(const ros::TimerEvent& event1);
    //void PrintEigenvalues(Eigen::EigenSolver<Eigen::Matrix<double,4,4>> es); //hardcoded 4x4
private:
    int solution_status_;
    double ro_ = 3;
    double R_ = 5;
    double w_ = 50;
    double cab_;
    double LP_solutions_[12];
    double moving_robot_linear_velocity_ = 0.2;
    double other_robots_velocity_ = 0.2;
    int k_ = 3; // k-hop factor
    Eigen::Vector3d all_fireflies_position_[4];
    Eigen::Matrix<double,4,4> connectivity_matrix_; //HARDCODED FOR 4 MAVs
    Eigen::Matrix<double,4,4> weighted_laplacian_matrix_; // -||-
    Eigen::Matrix<double,4,4> dFiedler_dL_matrix_;
    Eigen::Matrix<double,4,4> euclidian_distances_matrix_;
    Eigen::Matrix<double,4,4> A_kappa_matrix_;
    Eigen::Matrix<double,4,48> A_kappa_gradient_matrix_;
    Eigen::Matrix<double,1,12> k_hops_coefficients_;
    EigenStuff eigens_array_[4];
    Eigen::Matrix<double,4,4> L_derivatives_[12]; //Array of L derivatives over all coordinates of all robots - hardcode
    double trace_values_[12];
};

void OdometryWrapperWithAlgorithm::AlgorithmMainMethod() {
    double time_start =ros::Time::now().toNSec();
    ROS_INFO("*****time_start = %f",time_start);

    this->CalculateEucilidanDistancesMatrix();

    this->CalculateMatrices(); // Calculate connectivity and weighted Laplacian matrices
    //this->PrintConnectivityMatrix(4);
    //this->PrintWeightedLaplacian(4);

    Eigen::EigenSolver<Eigen::Matrix<double,4,4>> eigen_solver_instance(weighted_laplacian_matrix_);
    Eigen::Matrix<std::complex<double>,4,4> eigen_vectors = eigen_solver_instance.eigenvectors();;

    //ROS_INFO("-----Unsorted eigenvalues-----");
    for(int i = 0; i<4; i++) {
        eigens_array_[i].eigen_value_s = eigen_solver_instance.eigenvalues()[i].real();
        eigens_array_[i].eigen_vector_s = eigen_vectors.col(i);
    }
    std::sort(eigens_array_, eigens_array_ + 4, sortingFunction); //HARDCODED!
    //ROS_INFO("-----Sorted eigenvalues-----");

    this->CalculateFiedlerLaplacianDerivative();
    //this->PrintDFiedlerDL(4);

    this->CalculateLaplacianCoordinateDerivatives();
    this->CalculateTraceValues();


    // k-hop constraint stuff bellow
    Eigen::Matrix<double,1,12> cab_gradient = this->GetCabGradientAndFillCab_(2,3); // cab_ will get assigned
    k_hops_coefficients_ = -GetSigmoidDerivative(cab_)*cab_gradient; // negative because of the canonical LP form

    //double test_value = this->GetSigmoidDerivative(cab_);
    for(int i = 0; i<12; i++) {
        ROS_INFO("u'(cab_) = %f",GetSigmoidDerivative(cab_));
        ROS_INFO("---K-Hops Coeff[%d] = %f",i,-k_hops_coefficients_(i));
    }

    this->CalculateLP();

    //ROS_INFO("---EUCLIDIAN DISTANCES---");
    //this->Print44DoubleMatrix(euclidian_distances_matrix_);
    /*double time_finish =ros::Time::now().toNSec();
    ROS_INFO("*****time_finish = %f",time_finish);


    double time_diff = time_finish - time_start;
    ROS_INFO("*****time_diff = %f",time_diff);*/
}




void OdometryWrapperWithAlgorithm::CalculateMatrices() {

    //ROS_INFO("----------------aaaaaaaaaaaaa------------------------");
    //ROS_INFO("firefly1_position_ = (%f,%f,%f)",all_fireflies_position_[0](0),all_fireflies_position_[0](1),all_fireflies_position_[0](2));

    // Calculating the connectivity matrix
    for (int i = 0; i<4; i++) {
        for(int j = 0; j<4; j++) {
            double euclidian_distance = sqrt(pow((all_fireflies_position_[i](0)-all_fireflies_position_[j](0)),2) + pow((all_fireflies_position_[i](1)-all_fireflies_position_[j](1)),2) + pow((all_fireflies_position_[i](2)-all_fireflies_position_[j](2)),2));
            //ROS_INFO("Euclidian distance (%i,%i) = %f",i,j,euclidian_distance);
            if (euclidian_distance < ro_) {
                connectivity_matrix_(i,j) = 1;
            }
            else if (euclidian_distance >= R_) {
                connectivity_matrix_(i,j) = 0;
            }
            else {
                connectivity_matrix_(i,j) = exp((-5*(euclidian_distance - ro_))/(R_- ro_));
            }
            //ROS_INFO("Connectivity matrix (%i,%i) = %f",i,j,connectivity_matrix_(i,j));
        }
    }

    // Calculating the weighted Laplacian matrix
    for(int i = 0; i<4; i++) {
        for(int j = 0; j<4; j++) {
            if (i==j) {
                double sum = 0;
                for(int k = 0; k<4; k++) {
                    if (k!=i) {
                        sum = sum + connectivity_matrix_(i,k);
                    }
                }
                weighted_laplacian_matrix_(i,j) = sum;
            }
            else {
                weighted_laplacian_matrix_(i,j) = -connectivity_matrix_(i,j);
            }
            //ROS_INFO("Weighted Laplacian matrix (%i,%i) = %f",i,j,weighted_laplacian_matrix_(i,j));
        }
    }
}

void OdometryWrapperWithAlgorithm::CalculateFiedlerLaplacianDerivative() {
    double denominator=0;
    for(int i = 0; i<4; i++) {
        denominator = denominator + eigens_array_[1].eigen_vector_s(i).real() *eigens_array_[1].eigen_vector_s(i).real();
    }

    for(int i = 0; i<4; i++) {
        for(int j = 0; j<4; j++) {
            dFiedler_dL_matrix_(i,j) = (eigens_array_[1].eigen_vector_s(i).real() * eigens_array_[1].eigen_vector_s(j).real())/denominator;
        }
    }
}

void OdometryWrapperWithAlgorithm::CalculateLaplacianCoordinateDerivatives() {
    // L[0],L[1],L[2] will contain 0th robot movement instructions
    for(int i = 0; i<12; i++) {
        L_derivatives_[i].setZero();
    }

    // Calculate 12 L matrices (L_derivatives_)
    int counter = 0;
    for (int robot_counter = 0; robot_counter <4; robot_counter++) {
        for(int robot_coordinate_counter = 0; robot_coordinate_counter<3 ; robot_coordinate_counter++) {
            for(int i = 0; i<4; i++) {
                for(int j = 0; j<4; j++) {
                    if (i == j) {
                        L_derivatives_[counter](i,j) = 0;
                    }
                    else if (robot_counter==i || robot_counter==j) {
                        int other_interacting_robot;
                        if(robot_counter==i) {
                            other_interacting_robot = j; // fij derivative over Xzv, exists iff it is diagonal or on the "+" associated with that element
                        }
                        else if (robot_counter==j) {
                            other_interacting_robot = i;
                        }

                        // next "if" reflects the fact that derivative of a constant is zero
                        if (std::abs(weighted_laplacian_matrix_(i,j)+1) < 0.001  || std::abs(weighted_laplacian_matrix_(i,j)) < 0.001) {
                            L_derivatives_[counter](i,j) = 0; // consider the floating-point arit.
                        }
                        else {
                            L_derivatives_[counter](i,j) = -exp((-5*(euclidian_distances_matrix_(i,j) - ro_))/(R_- ro_))*(-5/(R_ - ro_))*((all_fireflies_position_[robot_counter](robot_coordinate_counter) - all_fireflies_position_[other_interacting_robot](robot_coordinate_counter)))/(euclidian_distances_matrix_(i,j));
                        }
                    }
                    else {
                        L_derivatives_[counter](i,j) = 0;
                    }
                    //L_derivatives_[counter](i,i) = L_derivatives_[counter](i,i) - L_derivatives_[counter](i,j)
                    //ROS_INFO("(%d,%d) = %f",i,j,L_derivatives_[counter](i,j));
                }
            }
            //ROS_INFO("---End of the matrix L[%d] robot counter[%d]---",counter,robot_counter);
            counter++;
        }
    }

    // Calculating the diagonal elements
    // Switch signs when calculating the sum on diagonal
    for(int laplacian_number = 0; laplacian_number<12; laplacian_number++) {
        Eigen::Matrix<double,4,4> L_derivatives_temp = -L_derivatives_[laplacian_number];
        for(int i = 0; i<4; i++) {
            L_derivatives_[laplacian_number](i,i) = L_derivatives_temp.row(i).sum();
        }
    }
}

void OdometryWrapperWithAlgorithm::CalculateEucilidanDistancesMatrix() {
    for(int i = 0; i<4; i++) {
        for(int j = 0; j<4; j++) {
            euclidian_distances_matrix_(i,j) = this->EuclidianDistanceFromTheOrigin(all_fireflies_position_[i] - all_fireflies_position_[j]);
            //ROS_INFO("Distance between %d and %d is %f",i,j,euclidian_distances_matrix_(i,j));
        }
    }
}

void OdometryWrapperWithAlgorithm::CalculateTraceValues() {
    Eigen::Matrix<double,4,4> temp_matrices[12];
    for(int i = 0; i<12; i++) {
        temp_matrices[i] = (dFiedler_dL_matrix_.transpose()) *L_derivatives_[i];
        trace_values_[i] = temp_matrices[i].trace();
        //ROS_INFO("trace_values[%d] = %f",i,trace_values_[i]);
        //ROS_INFO("------------------------------------------------");
    }
}

void OdometryWrapperWithAlgorithm::CalculateAKappaMatrix() {
    for(int i = 0; i<4; i++) {
        for(int j = 0; j<4; j++) {
            A_kappa_matrix_(i,j) = this->GetSigmoid(R_- euclidian_distances_matrix_(i,j));
        }
    }
}

void OdometryWrapperWithAlgorithm::CalculateAKappaGradientMatrix() {
    // every Laplacian-like-dimension-wise matrix entry should be swaped with a 12 row vector (gradient vector);
    /*for(int i = 0; i<4; i++) {
        int ii = 0;
        for(int j = 0; j<48; j=j+12) { // this iterates 4 times
            Eigen::Matrix<double,1,12> temp = this->Get_axy_Gradient(i,ii);
            ii++;
            for(int counter = 0; counter<12; counter++) {
                A_kappa_gradient_matrix_(i,j+counter) =  temp(1,counter);
            }
        }
    }*/
    for(int i = 0; i<4;i++) {
        for(int j = 0; j<4; j++) {
            A_kappa_gradient_matrix_.block(i,j*12,1,12) = this->Get_axy_Gradient(i,j);
        }
    }
}

void OdometryWrapperWithAlgorithm::CalculateLP() {
    cv::Mat z; // here a solution will be stored here
    cv::Mat A_LP; // [A b] matrix from LP will be stored here
    cv::Mat b(24,1,CV_64F,cv::Scalar(other_robots_velocity_)); //

    double data_k_hops[25];
    double data_c[24];
    for(int i = 0; i<24;i++) { // 12 traces values but because of nonnegativity substitution its 24
        if (i <=11) {
            data_c[i] = trace_values_[i];
            data_k_hops[i] = k_hops_coefficients_(i);
        }
        else {
            data_c[i] = -trace_values_[i-12];
            data_k_hops[i] = -k_hops_coefficients_(i-12);
        }
    }

    cv::Mat c = cv::Mat(1, 24, CV_64F, &data_c);

    data_k_hops[24] = 0.0;
    cv::Mat k_hops_coeffs_mat = cv::Mat(1, 25, CV_64F, &data_k_hops);

    cv::Mat In(12,12,CV_64F), In_negative(12,12,CV_64F);//, In_negative(12,12);
    In = cv::Mat::eye(In.rows, In.cols, CV_64F);
    In_negative = -cv::Mat::eye(In_negative.rows, In_negative.cols, CV_64F);
    cv::Mat m1; // helpful matrices for initialazing A_LP
    cv::Mat m2;

    cv::hconcat(In,In_negative,m1);
    cv::hconcat(In_negative,In,m2);
    cv::vconcat(m1,m2,A_LP);
    cv::hconcat(A_LP,b,A_LP);

    // LP constraint for the moving robot
    cv::Mat moving_robot_constraints(6,25,CV_64F,cv::Scalar(0));
    moving_robot_constraints.at<double>(0,9) = 1.0;
    moving_robot_constraints.at<double>(0,21) = -1.0;
    moving_robot_constraints.at<double>(1,9) = -1.0;
    moving_robot_constraints.at<double>(1,21) = 1.0;
    moving_robot_constraints.at<double>(2,10) = 1.0;
    moving_robot_constraints.at<double>(2,22) = -1.0;
    moving_robot_constraints.at<double>(3,10) = -1.0;
    moving_robot_constraints.at<double>(3,22) = 1.0;
    moving_robot_constraints.at<double>(4,11) = 1.0;
    moving_robot_constraints.at<double>(4,23) = -1.0;
    moving_robot_constraints.at<double>(5,11) = -1.0;
    moving_robot_constraints.at<double>(5,23) = 1.0;
    for(int i = 0; i<6; i++) {
        if (i%2==0) {
            moving_robot_constraints.at<double>(i,24) = moving_robot_linear_velocity_;
        }
        else {
            moving_robot_constraints.at<double>(i,24) = -moving_robot_linear_velocity_;
        }
    }

    // LP constraint for the static robot
    /*cv::Mat static_robot_zero_constraints(6,25,CV_64F,cv::Scalar(0));
    static_robot_zero_constraints.at<double>(0,6) = 1.0;
    static_robot_zero_constraints.at<double>(1,6) = -1.0;
    static_robot_zero_constraints.at<double>(2,7) = 1.0;
    static_robot_zero_constraints.at<double>(3,7) = -1.0;
    static_robot_zero_constraints.at<double>(4,8) = 1.0;
    static_robot_zero_constraints.at<double>(5,8) = -1.0;
    for(int i = 0; i<6; i++) {
        static_robot_zero_constraints.at<double>(i,24) = 0;
    }*/

    cv::vconcat(A_LP,moving_robot_constraints,A_LP);
    cv::vconcat(A_LP,k_hops_coeffs_mat,A_LP);
    //cv::vconcat(A_LP,static_robot_zero_constraints,A_LP);

    /*for(int i = 0; i<moving_robot_constraints.rows; i++) {
        for(int j = 0; j<moving_robot_constraints.cols; j++) {
            ROS_INFO("added cons (%d,%d) = %f",i,j,moving_robot_constraints.at<double>(i,j));
        }
    }*/
    /*ROS_INFO("Printing zeros I hope!");
    cv::MatIterator_<double> _it = someMatObject[2].begin<double>();
    for(;_it!=z.end<double>(); _it++){
        std::cout << *_it << std::endl;
    }*/

    int t;
    t = cv::solveLP(c,A_LP,z);
    ROS_INFO("solution status = %d",t); // ADD BEHAVIOUR IF THERE IS NO SOLUTION KEEP WHATHEVER YOU ARE DOING UNTIL THERE IS?!?!

    // setting real results because the problem is reduced to a canonical LP form
    for(int i = 0; i<12; i++) {
        LP_solutions_[i] = z.at<double>(i,0) - z.at<double>(i+12,0);
        //ROS_INFO("trace_values_[%d] = %f",i,trace_values_[i]);
        ROS_INFO("LP_solutions_[%d] = %f",i,LP_solutions_[i]);
        if (abs(LP_solutions_[i])>6) {
            ROS_INFO("*****************************************************************+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        }
    }
}



double* OdometryWrapperWithAlgorithm::GetTraceValues() {

    return trace_values_;
}

double* OdometryWrapperWithAlgorithm::GetLPsolutions() {

    return LP_solutions_;
}

double OdometryWrapperWithAlgorithm::GetSigmoid(double x) {
    double result = 1/(1+std::exp(-w_*x));
    return result;
}

double OdometryWrapperWithAlgorithm::GetSigmoidDerivative(double x)  {
    double result = (w_*std::exp(-w_*x)) / ((1 + std::exp(-w_*x))*(1 + std::exp(-w_*x)));
    return result;
}

Eigen::Matrix<double,1,12> OdometryWrapperWithAlgorithm::Get_axy_Gradient(int x, int y) {
    // 2 loops over all possible coordinates of all possible robots r = robot, c = coordinate
    Eigen::Matrix<double,1,12> axy_gradient;
    int counter = 0;
    //ROS_INFO("---PRINTING FOR (x,y) = (%d,%d)",x,y);
    for(int r=0; r<4; r++) {
        for(int c=0; c<3; c++) {
            if (r==x ^ r==y) { //cases for nonzero values, using xor operator to skip aii cases
                if (r==x) {
                    int other_interacting_robot = y;
                    axy_gradient(counter) = -(this->GetSigmoidDerivative(R_- euclidian_distances_matrix_(x,y))*(all_fireflies_position_[r](c)-all_fireflies_position_[other_interacting_robot](c)))/euclidian_distances_matrix_(x,y);
                    //ROS_INFO("---POTENTIAN FOR NAN =?= %f",euclidian_distances_matrix_(x,y));
                }
                else if (r==y) {
                    int other_interacting_robot = x;
                    axy_gradient(counter) = -(this->GetSigmoidDerivative(R_- euclidian_distances_matrix_(x,y))*(all_fireflies_position_[r](c)-all_fireflies_position_[other_interacting_robot](c)))/euclidian_distances_matrix_(x,y);
                    //ROS_INFO("---POTENTIAN FOR NAN =?= %f",euclidian_distances_matrix_(x,y));
                }
            }
            else { // zero in all but 6 cases (2 interacting robots * 3 coordinates)
                axy_gradient(0,counter) = 0;
            }
            //ROS_INFO("---da%d%d/dx%d%d = %f",x,y,r,c,axy_gradient(counter));
            counter++;
        }
    }
    //ROS_INFO("--------------------------------");
    return axy_gradient;
}

Eigen::Matrix<double,48,48> OdometryWrapperWithAlgorithm::GetKroneckerProduct(Eigen::Matrix<double,4,4> m1, Eigen::Matrix<double,12,12> m2) {
    Eigen::Matrix<double,48,48> m3;
    for (int i = 0; i < m1.cols(); i++) {
        for (int j = 0; j < m1.rows(); j++) {
            m3.block(i*m2.rows(), j*m2.cols(), m2.rows(), m2.cols()) =  m1(i,j)*m2;
        }
    }
    return m3;
}

Eigen::Matrix<double,1,12> OdometryWrapperWithAlgorithm::GetCabGradientAndFillCab_(int x, int y) {
    Eigen::Matrix<double,4,4> C_connectivity_matrices[k_+1], A_powers[k_+1];
    Eigen::Matrix<double,4,48> C_gradient_matrix = Eigen::Matrix<double,4,48>::Zero();
    Eigen::Matrix<double,12,12> temp_identity_matrix = Eigen::Matrix<double,12,12>::Identity();

    Eigen::Matrix<double,1,12> dummy_matrix = Eigen::Matrix<double,1,12>::Identity();

    this->CalculateAKappaMatrix(); // result in A_kappa_matrix_
    //ROS_INFO("-----A KAPPA MATRIX-----");
    //this->Print44DoubleMatrix(A_kappa_matrix_);

    this->CalculateAKappaGradientMatrix(); // result in A_kappa_gradient matrix_

    Eigen::Matrix<double,4,4> temp_A, temp_C;
    temp_A = Eigen::Matrix<double, 4, 4>::Identity();
    temp_C = Eigen::Matrix<double, 4, 4>::Identity();

    for(int i = 0; i<k_+1; i++) { //filling C_connectivity_matrices and A_powers matrices
        A_powers[i] = temp_A;
        //ROS_INFO("--- A^%d ---",i);
        //this->Print44DoubleMatrix(A_powers[i]);
        C_connectivity_matrices[i] = temp_C;
        temp_A = temp_A*A_kappa_matrix_;
        temp_C = temp_C + temp_A;
    }

    cab_ = C_connectivity_matrices[k_](x,y);

    for(int i = 0; i<=k_ - 1; i++) {
        C_gradient_matrix = C_gradient_matrix + C_connectivity_matrices[k_-1-i]*A_kappa_gradient_matrix_*this->GetKroneckerProduct(A_powers[i],temp_identity_matrix);
    }
    return C_gradient_matrix.block(x,y*12,1,12);
    //return dummy_matrix;
}

double OdometryWrapperWithAlgorithm::EuclidianDistanceFromTheOrigin(Eigen::Vector3d v) {

    return v.norm();
}

void OdometryWrapperWithAlgorithm::OdometryFirefly1MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    double x, y, z;
    x = odometry_msg->pose.pose.position.x;
    y = odometry_msg->pose.pose.position.y;
    z = odometry_msg->pose.pose.position.z;
    all_fireflies_position_[0] = Eigen::Vector3d(x, y, z);
}

void OdometryWrapperWithAlgorithm::OdometryFirefly2MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    double x, y, z;
    x = odometry_msg->pose.pose.position.x;
    y = odometry_msg->pose.pose.position.y;
    z = odometry_msg->pose.pose.position.z;
    all_fireflies_position_[1] = Eigen::Vector3d(x, y, z);
}

void OdometryWrapperWithAlgorithm::OdometryFirefly3MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    double x, y, z;
    x = odometry_msg->pose.pose.position.x;
    y = odometry_msg->pose.pose.position.y;
    z = odometry_msg->pose.pose.position.z;
    all_fireflies_position_[2] = Eigen::Vector3d(x, y, z);
}

void OdometryWrapperWithAlgorithm::OdometryFirefly4MethodCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    double x, y, z;
    x = odometry_msg->pose.pose.position.x;
    y = odometry_msg->pose.pose.position.y;
    z = odometry_msg->pose.pose.position.z;
    all_fireflies_position_[3] = Eigen::Vector3d(x, y, z);
}

void OdometryWrapperWithAlgorithm::PrintWeightedLaplacian(int laplacian_dimension) {
    ROS_INFO("-----LAPLACIAN MATRIX-----");
    for(int i = 0; i<laplacian_dimension; i++) {
        for(int j= 0; j<laplacian_dimension; j++) {
            ROS_INFO("(%i,%i) = %f",i,j,weighted_laplacian_matrix_(i,j));
        }
    }
    ROS_INFO("----------------------------------");
}

void OdometryWrapperWithAlgorithm::PrintConnectivityMatrix(int connectivity_matrix_dimension) {
    ROS_INFO("-----CONNECTIVITY MATRIX-----");
    for(int i = 0; i<connectivity_matrix_dimension; i++) {
        for(int j= 0; j<connectivity_matrix_dimension; j++) {
            ROS_INFO("(%i,%i) = %f",i,j,connectivity_matrix_(i,j));
        }
    }
    ROS_INFO("----------------------------------");
}

void OdometryWrapperWithAlgorithm::PrintDFiedlerDL(int dF_dL_dimension) {
    ROS_INFO("-----dFiedler/dL-----");
    for(int i = 0; i<dF_dL_dimension; i++) {
        for(int j= 0; j<dF_dL_dimension; j++) {
            ROS_INFO("(%i,%i) = %f",i,j,dFiedler_dL_matrix_(i,j));
        }
    }
    ROS_INFO("----------------------------------");
}

void OdometryWrapperWithAlgorithm::Print44DoubleMatrix(Eigen::Matrix<double,4,4> m) {
    for(int i = 0; i<4; i++) {
        for(int j= 0; j<4; j++) {
            ROS_INFO("(%i,%i) = %f",i,j,m(i,j));
        }
    }
    ROS_INFO("----------------------------------");
}

/*void OdometryWrapperWithAlgorithm::PrintLaplaciansOverCoordinatesMatrices(const ros::TimerEvent& event1) {
    ROS_INFO("??????????????????????????????????????????");
    for(int k = 0; k++; k<12) {
        for(int i = 0; i<4; i++) {
            for(int j= 0; j<4; j++) {
                ROS_INFO("(%d,%d) = %f",i,j,L_derivatives_[k](i,j));
            }
        }
        ROS_INFO("----------------------------------");
    }
} */

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_control_K_node");
  ros::NodeHandle nh;

  //rotors_gazebo::Num custom_float_array;

  OdometryWrapperWithAlgorithm odometry_wrapper_with_algorithm;

    //ros::Timer timer = nh.createTimer(ros::Duration(0.25), &OdometryWrapperWithAlgorithm::AlgorithmMainMethod,
    //&odometry_wrapper_with_algorithm);

  // Adding the subscribers to receive the odometry data from all fireflies (HARD CODED FOR 4 FIREFLIES)
  ros::Subscriber odometry_subscriber_firefly1 = nh.subscribe("/firefly1/odometry_sensor1/odometry", 1,
    &OdometryWrapperWithAlgorithm::OdometryFirefly1MethodCallback, &odometry_wrapper_with_algorithm);
  ros::Subscriber odometry_subscriber_firefly2 = nh.subscribe("/firefly2/odometry_sensor1/odometry", 1,
    &OdometryWrapperWithAlgorithm::OdometryFirefly2MethodCallback, &odometry_wrapper_with_algorithm);
  ros::Subscriber odometry_subscriber_firefly3 = nh.subscribe("/firefly3/odometry_sensor1/odometry", 1,
    &OdometryWrapperWithAlgorithm::OdometryFirefly3MethodCallback, &odometry_wrapper_with_algorithm);
  ros::Subscriber odometry_subscriber_firefly4 = nh.subscribe("/firefly4/odometry_sensor1/odometry", 1,
    &OdometryWrapperWithAlgorithm::OdometryFirefly4MethodCallback, &odometry_wrapper_with_algorithm);

  ros::Publisher array_pub = nh.advertise<rotors_gazebo::Num>("array", 100);


  ROS_INFO("Started global_control_K_node.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  ros::Rate loop_rate(20);
  while(ros::ok()){

    /*double time_start =ros::Time::now().toSec();
    ROS_INFO("*****time_start = %f",time_start);*/

    rotors_gazebo::Num array;
    array.data.clear();

    //try {
    odometry_wrapper_with_algorithm.AlgorithmMainMethod();
    //}
    /*catch(...) {
        continue;
    }*/

    double* temp = odometry_wrapper_with_algorithm.GetLPsolutions();
    for(int i = 0; i<12; i++) {
        array.data.push_back(temp[i]);
    }
    //array.data.push_back(temp[i]);
    array_pub.publish(array);
    ros::spinOnce();
    loop_rate.sleep();

    /*double time_finish =ros::Time::now().toSec();
    ROS_INFO("*****time_finish = %f",time_finish);

    double time_diff = time_finish - time_start;
    ROS_INFO("*****time_diff = %f",time_diff);*/


    ROS_INFO("----END OF THE MAIN METHOD----");
}
return 0;
}
