#ifndef UTILS_UTILS_H
#define UTILS_UTILS_H

#include <iostream>
#include <chrono>
#include <random>
#include <cmath>
#include <memory>
#include <algorithm>
#include <assert.h>
#include <iomanip>
#include <complex>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// in linux
#include <sys/io.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigenVec3;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigenVec3d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3i>> VecEigenVec3i;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VecEigenVector4d;
typedef std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> VecEigenVectorXf;
typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> VecEigenVectorXd;
typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> VecEigenMatrix3d;
typedef std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> VecEigenMatrixXd;
typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VecEigenIsometry3d;

class Utils
{
public:
    static std::default_random_engine rng;
    static int id;

    // -------------------------------------------------------
    /**
     * @param path: must end with "/"
     */
    static void createDirectory(
        const std::string &path)
    {
        int len = path.length();
        char tmpDirPath[256] = {0};
        for (int i = 0; i < len; i++)
        {
            tmpDirPath[i] = path[i];
            if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
            {
                if (access(tmpDirPath, 0) == -1)
                {
                    int ret = mkdir(tmpDirPath, 0777);
                    if (ret == -1)
                        std::cerr << "cannot create directory: " << path << std::endl;
                }
            }
        }
        return;
    }

    // -------------------------------------------------------
    static bool ifFileExist(
        const std::string &path)
    {
        int len = path.length();
        char tmpDirPath[256] = {0};

        for (int i = 0; i < len; i++)
            tmpDirPath[i] = path[i];

        if (access(tmpDirPath, 0) == -1)
            return false;

        return true;
    }

    // ------------------------------------------------------------
    template <typename T>
    static Eigen::VectorXd stdVector2EigenVectorXd(
        const std::vector<T> vector)
    {

        Eigen::VectorXd eigen(vector.size());
        for (int i = 0; i < vector.size(); i++)
        {
            eigen(i) = vector[i];
        }
        return eigen;
    }

    // ------------------------------------------------------------
    static std::vector<double> eigenVectorXd2StdVector(
        const Eigen::VectorXd &eigen)
    {
        std::vector<double> vector(eigen.rows());
        for (int i = 0; i < vector.size(); i++)
        {
            vector[i] = eigen(i);
        }
        return vector;
    }

    // ------------------------------------------------------------
    static std::vector<double> eigenVector2StdVector(
        const Eigen::Vector3d &eigen)
    {

        std::vector<double> vector(3);
        for (int i = 0; i < vector.size(); i++)
        {
            vector[i] = eigen(i);
        }
        return vector;
    }

    // ------------------------------------------------------------
    static std::vector<double> eigenVector2StdVector(
        const Eigen::Quaterniond &eigen)
    {

        std::vector<double> vector(4);
        for (int i = 0; i < vector.size(); i++)
        {
            vector[i] = eigen.coeffs()(i); // x, y, z, w
        }
        return vector;
    }

    // ------------------------------------------------------------
    static std::vector<double> eigenMatrix2StdVector(
        const Eigen::MatrixXd &eigen)
    {

        std::vector<double> vector(eigen.size());
        int k = 0;
        for (int i = 0; i < eigen.rows(); i++)
        {
            for (int j = 0; j < eigen.cols(); j++)
            {
                vector[k] = eigen(i, j);
                k++;
            }
        }
        return vector;
    }

    // ------------------------------------------------------------
    static std::vector<float> stdVectorDouble2Float(
        const std::vector<double> &d)
    {
        std::vector<float> f(d.begin(), d.end());
        return f;
    }

    // ------------------------------------------------------------
    static VecEigenVec3 eigenVectorXd2StdVecEigenVec3(const Eigen::VectorXd &vec)
    {
        if (vec.size() % 3 != 0)
        {
            std::cerr << "Utils::eigenVectorXd2StdVecEigenVec3(): input vector size is not the multiple of 3." << std::endl;
        }

        int num_fps = vec.size() / 3;
        VecEigenVec3 new_vec(num_fps);
        for (int i = 0; i < num_fps; i++)
        {
            new_vec[i] = vec.block<3, 1>(3 * i, 0);
        }
        return new_vec;
    }

    // ------------------------------------------------------------
    static Eigen::VectorXd stdVecEigenVec3ToEigenVectorXd(const VecEigenVec3 &std_vec)
    {
        int num_fps = std_vec.size();

        Eigen::VectorXd new_vec(num_fps * 3);
        for (int i = 0; i < num_fps; i++)
        {
            new_vec.block<3, 1>(3 * i, 0) = std_vec[i];
        }
        return new_vec;
    }

    // ------------------------------------------------------------
    static Eigen::Isometry3d stdPosQuatVec2Isometry3d(
        const std::vector<double> &pos,
        const std::vector<double> &quat // [x, y, z, w]
    )
    {
        return EigenPosQuatVec2Isometry3d(
            stdVector2EigenVectorXd(pos),
            stdVector2EigenVectorXd(quat));
    }

    // ------------------------------------------------------------
    static Eigen::Isometry3d EigenPosQuatVec2Isometry3d(
        const Eigen::Vector3d &pos,
        const Eigen::Vector4d &quat // [x, y, z, w]
    )
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        // normalize the quaternion to avoid invalid input quaternion
        Eigen::Quaternion<double> eigen_quat = Eigen::Quaternion<double>(quat).normalized(); // initialized from 4D vector [x, y, z, w]
        T.rotate(eigen_quat);
        T.pretranslate(pos);
        return T;
    }
    // ------------------------------------------------------------
    static Eigen::Isometry3d EigenPosQuatVec2Isometry3d(
        const Eigen::Vector3d &pos,
        const Eigen::Quaterniond &quat)
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(quat);
        T.pretranslate(pos);
        return T;
    }

    // ------------------------------------------------------------
    static double twoVecAngle(
        const Eigen::Vector3d &vec0,
        const Eigen::Vector3d &vec1)
    {
        return std::atan2(vec0.cross(vec1).norm(), vec0.dot(vec1));
    }

    // ------------------------------------------------------------
    template <typename T>
    static void coutStdVector(
        const std::vector<T> &vec)
    {
        for (auto &e : vec)
        {
            std::cout << e << " ";
        }
        std::cout << std::endl;
    }

    // ------------------------------------------------------------
    static double meanStdVec(
        const std::vector<double> &vec)
    {
        double sum = 0.0;
        for (auto &e : vec)
        {
            sum += e;
        }
        return sum / vec.size();
    }

    // ------------------------------------------------------------
    static void calcMeanAndStdOfVector(
        const std::vector<double> &vec,
        double &mean,
        double &std)
    {
        double sum = 0.0;
        for (auto &e : vec)
        {
            sum += e;
        }

        mean = sum / vec.size();

        double sum_var = 0.0;
        for (auto &e : vec)
        {
            sum_var += std::pow(e - mean, 2);
        }
        std = std::sqrt(sum_var / vec.size());
    }

    // ------------------------------------------------------------
    static void meanVecAndCovMatrix(
        const VecEigenVectorXd &data,
        Eigen::VectorXd &mean_vec,
        Eigen::MatrixXd &cov_mat)
    {
        int n_data = data.size();
        int dim = data[0].size();

        // calculate the mean vector
        mean_vec = Eigen::VectorXd::Zero(dim);
        for (size_t i = 0; i < n_data; i++)
        {
            mean_vec += data[i];
        }
        mean_vec /= n_data;

        // create the data matrix (n_data, dim)
        Eigen::MatrixXd data_mat = Eigen::MatrixXd::Zero(n_data, dim);
        for (size_t i = 0; i < n_data; i++)
        {
            data_mat.row(i) = data[i].transpose();
        }

        // 去均值
        for (size_t j = 0; j < dim; j++)
        {
            for (size_t i = 0; i < n_data; i++)
            {
                data_mat(i, j) -= mean_vec(j);
            }
        }

        // calculate the covariance matrix
        cov_mat = 1.0 / (n_data - 1) * data_mat.transpose() * data_mat;
    }

    // ------------------------------------------------------------
    static void coutList(
        const std::string &vec_name,
        const Eigen::VectorXd &vec)
    {
        std::cout << vec_name << " = [";
        for (int i = 0; i < vec.size(); i++)
        {
            std::cout << std::setprecision(4) << vec(i) << ", ";
        }
        std::cout << "]" << std::endl;
    }

    // ------------------------------------------------------------
    static void pushBackEigenVecToStdVec(
        std::vector<double> &vec,
        const Eigen::VectorXd &eigen_vec)
    {
        for (int j = 0; j < eigen_vec.size(); j++)
        {
            vec.push_back(eigen_vec(j));
        }
    }

    // ------------------------------------------------------------
    template <typename T>
    static void stdVecExpand(
        std::vector<T> &vec,
        const std::vector<T> &vec1)
    {
        vec.insert(vec.end(), vec1.begin(), vec1.end());
    }

    // ------------------------------------------------------------
    static double getRandomDouble(
        const double lb = 0.0,
        const double ub = 1.0)
    {
        std::uniform_real_distribution<> uniform(lb, ub);
        return uniform(rng);
    }

    // ------------------------------------------------------------
    static int getRandomInt(
        const int lb,
        const int ub // [lb, ub]
    )
    {
        std::uniform_int_distribution<int> rand(lb, ub);
        return rand(rng);
    }

    // ------------------------------------------------------------
    static Eigen::MatrixXd pseudoInverse(
        const Eigen::MatrixXd &a,
        double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
        return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

    // ------------------------------------------------------------
    static Eigen::MatrixXd dampedLeastSquareInverse(
        const Eigen::MatrixXd &a,
        const double &lambda_m,
        const double &epsilon)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double min_singular_value = svd.singularValues().minCoeff();

        double lambda_square = min_singular_value >= epsilon ? 0 : (1 - std::pow(min_singular_value / epsilon, 2)) * std::pow(lambda_m, 2);

        return a.transpose() * (a * a.transpose() + lambda_square * Eigen::MatrixXd::Identity(a.rows(), a.rows())).inverse();
    }

    // ------------------------------------------------------------
    static Eigen::MatrixXd getNullSpaceProjectionMatrix(
        const Eigen::MatrixXd &A)
    {
        return Eigen::MatrixXd::Identity(A.cols(), A.cols()) - pseudoInverse(A) * A;
    }

    /** -----------------------------------------------------------
     * @brief rpy: X-Y-Z fixed-axis angle
     */
    static Eigen::VectorXd isometryToPosAndRPYAngle(
        const Eigen::Isometry3d &pose)
    {
        using namespace std;

        Eigen::VectorXd pose_vec(6);
        pose_vec.block<3, 1>(0, 0) = pose.translation();

        Eigen::Matrix3d rot_mat = pose.rotation();

        // reference: J J. Craig, Introduction to Robotics, Section 2.8
        pose_vec(4) = atan2(-rot_mat(2, 0), sqrt(rot_mat(0, 0) * rot_mat(0, 0) + rot_mat(1, 0) * rot_mat(1, 0))); // range: [-pi/2, pi/2]

        // euler angle singularity problem
        if (cos(pose_vec(4)) == 0.0)
        {
            std::cerr << "The input pose is singular for fixed-axis angle representation." << std::endl;
        }

        pose_vec(3) = atan2(rot_mat(2, 1) / cos(pose_vec(4)), rot_mat(2, 2) / cos(pose_vec(4))); // range: [-pi, pi]
        pose_vec(5) = atan2(rot_mat(1, 0) / cos(pose_vec(4)), rot_mat(0, 0) / cos(pose_vec(4))); // range: [-pi, pi]

        return pose_vec;
    }

    /** -----------------------------------------------------------
     * @brief rpy: X-Y-Z fixed-axis angle
     */
    static Eigen::Matrix3d matrixRelateAngularVelToRPYVel(
        const Eigen::Vector3d &rpy_vec)
    {
        using namespace std;

        double r = rpy_vec(0);
        double p = rpy_vec(1);
        double y = rpy_vec(2);

        Eigen::Matrix3d matrix;
        matrix << cos(y) / cos(p), sin(y) / cos(p), 0.0,
            -sin(y), cos(y), 0.0,
            cos(y) * sin(p) / cos(p), sin(y) * sin(p) / cos(p), 1.0;

        return matrix;
    }

    // ------------------------------------------------------------
    static double distanceBetweenTwoPose(
        const Eigen::Isometry3d &pose_0,
        const Eigen::Isometry3d &pose_1,
        const double pos_weight = 1.0,
        const double rot_weight = 1.0)
    {
        double pos_dist = (pose_1.translation() - pose_0.translation()).norm();
        Eigen::AngleAxisd rotation_vector(pose_1.rotation() * pose_0.rotation().transpose());
        double rot_angle = rotation_vector.angle();

        return pos_weight * pos_dist + rot_weight * rot_angle;
    }

    // ------------------------------------------------------------
    static double distanceBetweenTwoPose(
        const Eigen::VectorXd &se3_vec_0,
        const Eigen::VectorXd &se3_vec_1,
        const double pos_weight,
        const double rot_weight)
    {
        double pos_dist = (se3_vec_1.block<3, 1>(0, 0) - se3_vec_0.block<3, 1>(0, 0)).norm();

        Eigen::Vector3d rot_vec_0 = se3_vec_0.block<3, 1>(3, 0);
        Eigen::Vector3d rot_vec_1 = se3_vec_1.block<3, 1>(3, 0);

        Eigen::AngleAxisd angle_axis_0(rot_vec_0.norm(), rot_vec_0.normalized());
        Eigen::AngleAxisd angle_axis_1(rot_vec_1.norm(), rot_vec_1.normalized());

        double rot_angle = Eigen::AngleAxisd(angle_axis_1 * angle_axis_0.inverse()).angle();

        return pos_weight * pos_dist + rot_weight * rot_angle;
    }

    // ------------------------------------------------------------
    static Eigen::Quaterniond eulerAngleToQuat(
        const Eigen::Vector3d &angles,
        const Eigen::Vector3d &order)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

        for (int j = 0; j < 3; j++)
        {
            Eigen::Vector3d axis;
            if (order[j] == 0)
                axis = Eigen::Vector3d::UnitX();
            else if (order[j] == 1)
                axis = Eigen::Vector3d::UnitY();
            else if (order[j] == 2)
                axis = Eigen::Vector3d::UnitZ();
            else
                std::cerr << "Utils::eulerAngleToQuat(): invalid order." << std::endl;

            q = q * Eigen::AngleAxisd(angles[j], axis);
        }

        return q;
    }

    // ------------------------------------------------------------
    template <typename T>
    static std::vector<T> concatenateTwoVector(
        const std::vector<T> &vec1,
        const std::vector<T> &vec2)
    {
        std::vector<T> new_vec = vec1;
        new_vec.insert(new_vec.end(), vec2.begin(), vec2.end());
        return new_vec;
    }

    // ------------------------------------------------------------
    static Eigen::VectorXd concatenateTwoVector(
        const Eigen::VectorXd &vec1,
        const Eigen::VectorXd &vec2)
    {
        Eigen::VectorXd new_vec = Eigen::VectorXd::Zero(vec1.size() + vec2.size());
        new_vec.block(0, 0, vec1.size(), 1) = vec1;
        new_vec.block(vec1.size(), 0, vec2.size(), 1) = vec2;
        return new_vec;
    }

    // ------------------------------------------------------------
    static Eigen::VectorXd pose2se3Vec(
        const Eigen::Isometry3d &pose)
    {
        Eigen::AngleAxisd angle_axis(pose.rotation());

        Eigen::VectorXd se3_vec = Eigen::VectorXd::Zero(6);
        se3_vec.block<3, 1>(0, 0) = pose.translation();
        se3_vec.block<3, 1>(3, 0) = angle_axis.angle() * angle_axis.axis();

        return se3_vec;
    }

    // ------------------------------------------------------------
    static std::vector<double> eigenMatrixToStdVec(
        const Eigen::MatrixXd &mat)
    {
        std::vector<double> vec(mat.rows() * mat.cols());
        for (int row = 0; row < mat.rows(); row++)
        {
            for (int col = 0; col < mat.cols(); col++)
            {
                vec[mat.cols() * row + col] = mat(row, col);
            }
        }
        return vec;
    }

    // ------------------------------------------------------------
    static Eigen::MatrixXd stdVecToEigenMatrix(
        const std::vector<double> &vec,
        int n_row,
        int n_col)
    {
        if (n_row * n_col != vec.size())
            std::cerr << "The n_row and n_col do not match the vector size." << std::endl;

        Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n_row, n_col);
        for (int row = 0; row < n_row; row++)
        {
            for (int col = 0; col < n_col; col++)
            {
                mat(row, col) = vec[n_col * row + col];
            }
        }
        return mat;
    }

    // ---------------------------------------------------
    static std::vector<double> interpolate(
        const std::vector<double> &x_list,
        const std::vector<std::vector<double>> &y_list,
        const double x)
    {
        if (x < x_list[0] || x > x_list[x_list.size() - 1])
        {
            std::cerr << "Invalid x." << std::endl;
        }

        double id_1, id_2;
        for (int i = 1; i < x_list.size(); i++)
        {
            if (x <= x_list[i])
            {
                id_1 = i - 1;
                id_2 = i;
                break;
            }
        }

        double t = (x - x_list[id_1]) / (x_list[id_2] - x_list[id_1]);

        std::vector<double> y(y_list[0].size());
        for (int j = 0; j < y.size(); j++)
        {
            y[j] = y_list[id_1][j] * (1 - t) + y_list[id_2][j] * t;
        }

        return y;
    }

}; // end class

#endif