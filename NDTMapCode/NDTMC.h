//
// Created by liaolizhou on 23-4-3.
//

#ifndef MYPROJECT_NDTMC_H
#define MYPROJECT_NDTMC_H

#include <string>
#include <fstream>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

using namespace std;

class NDTMC {
    /*
     * 1. read PointCloud from KITTI .bin
     * 2. read true&false txt
     * 3. create NDT-Map-Code descriptor
     * 4. compare with others
     * */
public:
    using NDTGrid = typename pcl::VoxelGridCovariance<pcl::PointXYZI>;
    using Leaf = typename NDTGrid::Leaf;
    struct NDTCell
    {
        NDTCell(Eigen::Vector3d mean_, Eigen::Matrix3d cov) : cov_(std::move(cov))
        {
            point_.x = static_cast<float>(mean_(0));
            point_.y = static_cast<float>(mean_(1));
            point_.z = static_cast<float>(mean_(2));
        }
        pcl::PointXYZI point_;
        Eigen::Matrix3d cov_ = Eigen::Matrix3d::Identity();
    };
    using NDTCellPtr = NDTCell *;

    struct GridCell {
        int counts[8] = {0}; // 记录每个shape出现的次数
        int shape_max = -1;  // 记录出现次数最多的shape
        int count_max = 0;   // 记录出现次数最多的shape出现的次数

        void addShape(int shape) {
            counts[shape - 1]++;  // 根据shape值更新counts数组
            int total_counts = 0;
            for (int count : counts) {
                total_counts += count;
            }
            if (counts[shape - 1] > count_max && counts[shape - 1] >= double(total_counts) / 8.0) {  // 如果出现次数超过了之前最大值并且超过总次数的25%
                count_max = counts[shape - 1];
                shape_max = shape;
            } else if (count_max < double(total_counts) / 8.0) {  // 如果最多的shape数量没有超过25%
                shape_max = -1;
            }
        }
    };

    struct MergedGrid {
        int num_of_point = 0;
        bool is_first = true;
        Eigen::Vector3d mean_;
        Eigen::Matrix3d cov_;
        void addCell(const Eigen::Vector3d mean_in, const Eigen::Matrix3d& cov_in, const int point_number){
            if(is_first){
                mean_ = mean_in;
                cov_ = cov_in;
                num_of_point = point_number;
                is_first = false;
            }else{
                cov_ = (num_of_point - 1.0) / (num_of_point + point_number - 2.0) * cov_
                        + (point_number - 1.0) / (num_of_point + point_number - 2.0) * cov_in
                        + (num_of_point * point_number) / ((num_of_point + point_number) * (num_of_point + point_number - 1.0))
                          * (mean_ - mean_in) * (mean_ - mean_in).transpose();
                mean_ = (num_of_point * mean_ + point_number * mean_in) / (num_of_point + point_number);
                num_of_point += point_number;
            }
        }
    };



public:
    NDTMC(float resolution_);
    NDTMC();
    void readKittiBin(const string &in_file);
    std::map<std::size_t, Leaf> readNIOBin(const string &in_file);
    void readPCD(const string &in_file);
    static double distDirect(const Eigen::MatrixXd& _frame_sc, Eigen::MatrixXd _submap_sc);
    static Eigen::MatrixXd circshift(Eigen::MatrixXd _mat, const Eigen::MatrixXd& _mat2);
    static Eigen::MatrixXd circshift(Eigen::MatrixXd _mat);
    static vector<pair<double, int>> distanceBtnNDTScanContext(Eigen::MatrixXd& frame_sc, Eigen::MatrixXd& submap_sc);
    void getNDTLeaves(const std::map<std::size_t, Leaf> &leaves_);
    Eigen::MatrixXd getNDTLeaves_new(const std::map<std::size_t, Leaf> &leaves_);
    void transformToNDTForm(float resolution_);
    bool CalculateSectorId(pcl::PointXYZI &pt_in, int &ring_idx, int &sctor_idx);
    static float xy2theta(const float &_x, const float &_y);
    int CalculateLayer(float z) const;
    int GetDistributionWeight(int idx);
    int CalculateShapeWeight(const GridCell& cell);
    Eigen::MatrixXd createNDTMC();
    Eigen::MatrixXd createNDTMC(const std::map<std::size_t, NDTMC::Leaf>& leaves);
    Eigen::MatrixXd createNDTMC_KITTI();

    struct LoopResults {
        LoopResults(double a, int b, int c) : similarity(a), shift(b), frame_id(c){};
        double similarity;
        int shift;
        int frame_id;
    };

    struct GridBin {
        int num_of_points = 0;
        Eigen::MatrixXd Points;
        Eigen::RowVector3d mean;
        Eigen::Matrix3d cov;

        GridBin() {
            Points.resize(1, 3);
            Points.setZero();
        }

        void addPoint(float x_, float y_, float z_) {
            num_of_points++;
            if(num_of_points == 1){
                Eigen::RowVector3d p_new(static_cast<double>(x_), static_cast<double>(y_), static_cast<double>(z_));
                Points.conservativeResize(1, Eigen::NoChange);
                Points.row(Points.rows()-1) = p_new;
                mean = p_new;
                cov.setZero();
                return;
            }
            Eigen::RowVector3d p_new(static_cast<double>(x_), static_cast<double>(y_), static_cast<double>(z_));
            Points.conservativeResize(Points.rows()+1, Eigen::NoChange);
            Points.row(Points.rows()-1) = p_new;

            // Incrementally update mean and covariance matrix
            if (num_of_points > 1) {
                Eigen::RowVector3d mean_prev = mean;
                Eigen::Matrix3d cov_prev = cov;

                // Update mean incrementally
                mean += (p_new - mean_prev) / num_of_points;

                // Update covariance matrix incrementally
                cov = (num_of_points - 2) / static_cast<double>(num_of_points - 1) * cov_prev;
                cov += (p_new - mean_prev).transpose() * (p_new - mean) / static_cast<double>(num_of_points - 1);

                // Do something with the mean and covariance matrix
            } else {
                mean = p_new;
                cov.setZero();
            }
        }
    };

public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in;
    // ndt format
    NDTGrid voxel_grid_frame;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_pointcloud;
    vector<NDTCell> ndtcell;
    static const int num_intervals = 10;
    Eigen::Matrix<double,1,num_intervals> histogram;
    float resolution{};

    // parameter of scancontext
    const float LIDAR_HEIGHT = 1.73;
    const int PC_NUM_RING = 20;
    const int PC_NUM_SECTOR = 60;
    const int PC_NUM_Z = 6;

    const double PC_MAX_RADIUS = 80.0;
    static constexpr double PC_MAX_Z = 6;
//    const double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);


};


#endif //MYPROJECT_NDTMC_H
