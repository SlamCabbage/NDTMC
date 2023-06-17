#ifndef MY_VOXEL_GRID_COVARIANCE_H
#define MY_VOXEL_GRID_COVARIANCE_H
#include <pcl/filters/voxel_grid_covariance.h>

class MyVoxelGridCovariance : public pcl::VoxelGridCovariance<pcl::PointXYZI>
{
public:
    // 默认构造函数
    MyVoxelGridCovariance() {}

    bool loadVoxelGrid(std::ifstream &ifs)
    {
        this->voxel_centroids_ = PointCloudPtr(new PointCloud());
        this->voxel_centroids_->height = 1;
        this->voxel_centroids_->is_dense = true;
        this->voxel_centroids_->points.clear();

        // load grid size
        int min_b[4];
        ifs.read((char *)min_b, sizeof(int) * 4);
        this->min_b_ = Eigen::Map<Eigen::Vector4i>(min_b);
        int max_b[4];
        ifs.read((char *)max_b, sizeof(int) * 4);
        this->max_b_ = Eigen::Map<Eigen::Vector4i>(max_b);
        int div_b[4];
        ifs.read((char *)div_b, sizeof(int) * 4);
        this->div_b_ = Eigen::Map<Eigen::Vector4i>(div_b);
        int divb_mul[4];
        ifs.read((char *)divb_mul, sizeof(int) * 4);
        this->divb_mul_ = Eigen::Map<Eigen::Vector4i>(divb_mul);

        // load cell ndt params
        int leave_size = 0;
        ifs.read((char *)&leave_size, sizeof(int));
        PCL_INFO("load leaves number is %d\n", leave_size);
        this->voxel_centroids_leaf_indices_.clear();
        this->voxel_centroids_leaf_indices_.reserve(leave_size);
        this->voxel_centroids_->points.reserve(leave_size);
        // Clear the leaves
        this->leaves_.clear();
        for (size_t i = 0; i < leave_size; i++)
        {
            size_t index;
            ifs.read((char *)&index, sizeof(size_t));
            Leaf &leaf = this->leaves_[index];
            this->voxel_centroids_leaf_indices_.push_back(static_cast<int>(index));

            /** \brief Number of points contained by voxel */
            int num_point = 0;
            ifs.read((char *)&num_point, sizeof(int));
            leaf.nr_points = num_point;

            /** \brief 3D voxel mean and centroid */
            double mean[3];
            ifs.read((char *)mean, sizeof(double) * 3);
            leaf.mean_ = Eigen::Map<Eigen::Vector3d>(mean);
            leaf.centroid.resize(4);
            Eigen::Vector4f pt(mean[0], mean[1], mean[2], 0);
            leaf.centroid.template head<4>() = pt;

            this->voxel_centroids_->push_back(pcl::PointXYZI());
            this->voxel_centroids_->points.back().x = leaf.centroid[0];
            this->voxel_centroids_->points.back().y = leaf.centroid[1];
            this->voxel_centroids_->points.back().z = leaf.centroid[2];

            /** \brief Voxel covariance matrix */
            double cov[9];
            ifs.read((char *)cov, sizeof(double) * 9);
            leaf.cov_ = Eigen::Map<Eigen::Matrix3d>(cov);
            leaf.icov_ = leaf.cov_.inverse();

            /** \brief Eigen vectors of voxel covariance matrix */
            double evecs[9];
            ifs.read((char *)evecs, sizeof(double) * 9);
            leaf.evecs_ = Eigen::Map<Eigen::Matrix3d>(evecs);

            /** \brief Eigen values of voxel covariance matrix */
            double evals[3];
            ifs.read((char *)evals, sizeof(double) * 3);
            leaf.evals_ = Eigen::Map<Eigen::Vector3d>(evals);
        }

        this->voxel_centroids_->width = static_cast<std::uint32_t>(this->voxel_centroids_->points.size());
        if (!this->voxel_centroids_->empty())
        {
            this->kdtree_.setInputCloud(voxel_centroids_);
        }
        return true;
    }

    bool saveVoxelGrid(std::ofstream &fout)
    {
        // save leaf size
        float *leaf_size = this->leaf_size_.data();
        fout.write((char *)leaf_size, sizeof(float) * 4);

        // save grid size
        int *min_b = this->min_b_.data();
        fout.write((char *)min_b, sizeof(int) * 4);
        int *max_b = this->max_b_.data();
        fout.write((char *)max_b, sizeof(int) * 4);
        int *div_b = this->div_b_.data();
        fout.write((char *)div_b, sizeof(int) * 4);
        int *divb_mul = this->divb_mul_.data();
        fout.write((char *)divb_mul, sizeof(int) * 4);

        // save cell ndt params
        int leave_size = 0;
        for (typename std::map<std::size_t, Leaf>::iterator it = this->leaves_.begin(); it != this->leaves_.end(); ++it)
        {
            Leaf &leaf = it->second;
            if (leaf.nr_points < this->min_points_per_voxel_)
                continue;
            ++leave_size;
        }

        fout.write((char *)&leave_size, sizeof(int));
        PCL_INFO("save leaves number is %d\n", leave_size);
        for (typename std::map<std::size_t, Leaf>::iterator it = this->leaves_.begin(); it != this->leaves_.end(); ++it)
        {
            Leaf &leaf = it->second;
            if (leaf.nr_points < this->min_points_per_voxel_)
                continue;
            size_t index = it->first;
            fout.write((char *)&index, sizeof(size_t));

            /** \brief Number of points contained by voxel */
            int num_point = leaf.nr_points;
            fout.write((char *)&num_point, sizeof(int));

            /** \brief 3D voxel mean */
            double *mean = leaf.mean_.data();
            fout.write((char *)mean, sizeof(double) * 3);

            /** \brief Voxel covariance matrix */
            double *cov = leaf.cov_.data();
            fout.write((char *)cov, sizeof(double) * 9);

            /** \brief Eigen vectors of voxel covariance matrix */
            double *evecs = leaf.evecs_.data();
            fout.write((char *)evecs, sizeof(double) * 9);

            /** \brief Eigen values of voxel covariance matrix */
            double *evals = leaf.evals_.data();
            fout.write((char *)evals, sizeof(double) * 3);
        }
        return true;
    }

    map<size_t, Leaf> getNDTSubMap(Eigen::Matrix4d &pose, const int &radius)
    {
        vector<float> distances;
        pcl::PointXYZI position;
        position.x = pose(0, 3);
        position.y = pose(1, 3);
        position.z = pose(2, 3);
        Eigen::Matrix3d rot = pose.block<3,3>(0,0);
        vector<LeafConstPtr> k_leaves;
        Eigen::Matrix4d imu_To_car;
        imu_To_car << 0, -1, 0, 0.386, -1, 0, 0, 0.639, 0, 0, -1, 0.054, 0, 0, 0, 1;
        Eigen::Matrix3d rot1 = imu_To_car.block<3,3>(0,0);
        radiusSearch(position, radius, k_leaves, distances);
        map<size_t, Leaf> leaves;
        int i = 0;
        for (auto constleafptr : k_leaves)
        {
            Leaf leaf;
            leaf.nr_points = constleafptr->nr_points;
            Eigen::Matrix4d pose_inverse = pose.inverse();
            Eigen::Vector4d mean_homogeneous(constleafptr->mean_[0], constleafptr->mean_[1], constleafptr->mean_[2], 1.0);
            Eigen::Vector4d transformed_mean_homogeneous = pose_inverse * mean_homogeneous;
            transformed_mean_homogeneous = imu_To_car * transformed_mean_homogeneous;
            Eigen::Vector3d transformed_mean(transformed_mean_homogeneous[0], transformed_mean_homogeneous[1], transformed_mean_homogeneous[2]);
            leaf.mean_ = transformed_mean;
            if (transformed_mean[2] < 0 || transformed_mean[2] > NDTMC::PC_MAX_Z)
                continue;
            // Eigen::Matrix3d cov_temp = constleafptr->cov_;
            // cov_temp = rot.transpose() * cov_temp * rot;
            // cov_temp = rot1 * cov_temp * rot1.transpose();
            // leaf.cov_ = cov_temp;
            leaf.cov_ = constleafptr->cov_;
            leaf.icov_ = constleafptr->icov_;
            leaf.evecs_ = constleafptr->evecs_;
            leaf.evals_ = constleafptr->evals_;
            leaves.insert(make_pair(i++, leaf));
        }
        return leaves;
    }
};
#endif // MY_VOXEL_GRID_COVARIANCE_H