//
// Created by liaolizhou on 23-4-3.
//

#include "NDTMC.h"

NDTMC::NDTMC(float resolution_) : resolution(resolution_)
{
    ndt_pointcloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
};

void NDTMC::readPCD(const string &in_file)
{
    // 不需要修改高度，因为已经在car系
    pc_in = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *pc_in) == -1) //* load the file
    {
        cerr << "Couldn't read file " << in_file << ".pcd" << endl;
        return;
    }

    //    pcl::io::savePCDFileBinary("/home/liaolizhou/Myproject_copy/data/1.pcd",*pc_in);
}

// 读取存储的nio dataset submaps
std::map<std::size_t, NDTMC::Leaf> NDTMC::readNIOBin(const string &in_file)
{

    pc_in = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    string lidar_filename_path = in_file;
    ifstream inputfile;
    inputfile.open(lidar_filename_path, ios::binary);
    if (!inputfile)
    {
        cerr << "ERROR: Cannot open file " << lidar_filename_path
             << "! Aborting..." << endl;
        return {};
    }

    // read leaves_
    std::map<std::size_t, Leaf> leaves_;
    inputfile.seekg(0, ios::beg);
    for (std::size_t i = 0; !inputfile.eof(); i++)
    {
        Leaf leaf_;
        inputfile.read(reinterpret_cast<char *>(&leaf_.nr_points), sizeof(int));
        inputfile.read(reinterpret_cast<char *>(&leaf_.mean_), sizeof(Eigen::Vector3d));
        inputfile.read(reinterpret_cast<char *>(&leaf_.cov_), sizeof(Eigen::Matrix3d));
        inputfile.read(reinterpret_cast<char *>(&leaf_.icov_), sizeof(Eigen::Matrix3d));
        inputfile.read(reinterpret_cast<char *>(&leaf_.evecs_), sizeof(Eigen::Matrix3d));
        inputfile.read(reinterpret_cast<char *>(&leaf_.evals_), sizeof(Eigen::Vector3d));

        leaves_.insert(make_pair(i, leaf_));
    }
    return leaves_;
    //    pcl::io::savePCDFileBinary("/home/liaolizhou/Myproject_copy/data/1.pcd",*pc_in);
}

void NDTMC::readKittiBin(const string &in_file)
{
    pc_in = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    string lidar_filename_path = in_file;
    ifstream inputfile;
    inputfile.open(lidar_filename_path, ios::binary);
    if (!inputfile)
    {
        cerr << "ERROR: Cannot open file " << lidar_filename_path
             << "! Aborting..." << endl;
        return;
    }

    inputfile.seekg(0, ios::beg);
    for (int i = 0; inputfile.good() && !inputfile.eof(); i++)
    {
        pcl::PointXYZI p;
        inputfile.read((char *)&p.x, 3 * sizeof(float));
        inputfile.read((char *)&p.intensity, sizeof(float));
        p.z += LIDAR_HEIGHT;
        pc_in->points.push_back(p);
    }
    //    pcl::io::savePCDFileBinary("/home/liaolizhou/Myproject_copy/data/1.pcd",*pc_in);
}

double NDTMC::distDirect(const Eigen::MatrixXd &_frame_sc, Eigen::MatrixXd _submap_sc)
{
    // Calculate a_mean and b_mean
    double a_mean = _frame_sc.mean();
    double b_mean = _submap_sc.mean();

    // Center a_mix and b_mix around their means
    Eigen::MatrixXd a = _frame_sc.array() - a_mean;
    a = (a.array() == -a_mean).select(0, a); // replace -a_mean with 0
    Eigen::MatrixXd b = _submap_sc.array() - b_mean;
    b = (b.array() == -b_mean).select(0, b); // replace -b_mean with 0
    // const Eigen::MatrixXd &a = _frame_sc;
    // const Eigen::MatrixXd &b = _submap_sc;

    // // Calculate correlation similarity
    double sc_sim = (a.cwiseProduct(b).colwise().sum()).sum() /
                    sqrt((a.cwiseProduct(a).colwise().sum()).sum() * (b.cwiseProduct(b).colwise().sum()).sum());

    return 1.0 - abs(sc_sim);

} // distDirectSC

Eigen::MatrixXd NDTMC::circshift(Eigen::MatrixXd _mat, const Eigen::MatrixXd &_mat2)
{
    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols() + _mat2.cols());
    shifted_mat.block(0, 0, _mat.rows(), _mat.cols()) = _mat;
    shifted_mat.block(0, _mat.cols(), _mat.rows(), _mat2.cols()) = _mat.block(0, 0, _mat.rows(), _mat2.cols());
    return shifted_mat;
} // circshift

Eigen::MatrixXd NDTMC::circshift(Eigen::MatrixXd _mat)
{
    Eigen::MatrixXd expanded_mat = Eigen::MatrixXd::Zero(_mat.rows(), 2 * _mat.cols() - 1);
    expanded_mat.block(0, 0, _mat.rows(), _mat.cols()) = _mat;
    for (int i = 1; i < _mat.cols(); ++i)
    {
        expanded_mat.block(0, i + _mat.cols() - 1, _mat.rows(), 1) = _mat.block(0, i, _mat.rows(), 1);
    }
    return expanded_mat;
} // circshift

vector<pair<double, int>> NDTMC::distanceBtnNDTScanContext(Eigen::MatrixXd &frame_sc,
                                                           Eigen::MatrixXd &submap_sc)
{
    std::vector<std::pair<double, int>> results;
    results.emplace_back(1000.0, -1);
    int len_cols = static_cast<int>(submap_sc.cols());
    //    Eigen::MatrixXd submap_sc_ = circshift(submap_sc, frame_sc);
    Eigen::MatrixXd submap_sc_ = circshift(submap_sc);
    for (int num_shift = 0; num_shift < len_cols; ++num_shift)
    {
        Eigen::MatrixXd submap_sc_shifted = submap_sc_.block(0, num_shift, frame_sc.rows(), frame_sc.cols());
        double cur_sc_dist = distDirect(frame_sc, submap_sc_shifted);
        if (cur_sc_dist == 0)
        {
            continue;
        }
        if (cur_sc_dist != cur_sc_dist)
            continue;
        results.emplace_back(cur_sc_dist, num_shift);
    }
    return results;
} // distanceBtnNDTScanContext

void NDTMC::transformToNDTForm(float resolution_)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*pc_in);
    voxel_grid_frame.setLeafSize(resolution_, resolution_, resolution_);
    voxel_grid_frame.setInputCloud(cloudPtr);
    // Initiate voxel structure.
    voxel_grid_frame.filter(true);
} // transformToNDTForm

void NDTMC::getNDTLeaves(const std::map<std::size_t, Leaf> &leaves_)
{
    ndtcell.clear();
    ndt_pointcloud->clear();
    ofstream f, ff;
    ff.open("/home/liaolizhou/Myproject_copy/data/cov1.txt", ios::out);
    f.open("/home/liaolizhou/Myproject_copy/data/g1.txt", ios::out);
    for (const auto &leave : leaves_)
    {
        if (leave.second.nr_points < 5)
            continue;
        //********************************************
        Eigen::Matrix3d cov = leave.second.cov_;
        //        cov.normalize();
        cout << cov.norm() << endl;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov); // SelfAdjointEigenSolver计算特征向量和特征值
        double value = saes.eigenvalues()(2) * saes.eigenvalues()(0) / (saes.eigenvalues()(1) * saes.eigenvalues()(1));
        double distance =
            leave.second.mean_(0) * leave.second.mean_(0) + leave.second.mean_(1) * leave.second.mean_(1) +
            leave.second.mean_(2) * leave.second.mean_(2);
        Eigen::Vector3d z_normal(0, 0, 1);
        Eigen::Vector3d plane_normal_vector = saes.eigenvectors().col(0);
        double angle = acos(plane_normal_vector.dot(z_normal) / (plane_normal_vector.norm())) * 180 / M_PI;
        //        cout << saes.eigenvalues()(2) << " " << saes.eigenvalues()(1) << " " << saes.eigenvalues()(0) << " " << angle
        //             << " " << value << endl;
        if (sqrt(distance) < PC_MAX_RADIUS && leave.second.mean_(2) < PC_MAX_Z && leave.second.mean_(2) > 0)
        {
            f << value << " ";
            ff << "mean:" << endl;
            ff << leave.second.mean_(0) << " " << leave.second.mean_(1) << " " << leave.second.mean_(2) << endl;
            ff << "cov:" << endl;
            ff << cov << endl;
        }

        //********************************************
        NDTCell temp_(leave.second.mean_, leave.second.cov_);

        pcl::PointXYZI point;
        point.x = static_cast<float>(leave.second.mean_(0));
        point.y = static_cast<float>(leave.second.mean_(1));
        point.z = static_cast<float>(leave.second.mean_(2));
        ndt_pointcloud->points.emplace_back(point);
        ndtcell.emplace_back(temp_);
    }
    ff.close();
    f.close();
} // getNDTLeaves

Eigen::MatrixXd NDTMC::getNDTLeaves_new(const std::map<std::size_t, Leaf> &leaves_)
{
    GridCell grid[PC_NUM_RING][PC_NUM_SECTOR][PC_NUM_Z];
    Eigen::MatrixXd desc_1(PC_NUM_RING, PC_NUM_SECTOR);
    desc_1.setZero();
    histogram.setZero();
    for (const auto &leave : leaves_)
    {
        if (leave.second.nr_points < 5 || leave.second.evals_(2) == 0){
            continue;
        }
        Eigen::Matrix3d cov = leave.second.cov_;
        pcl::PointXYZI point;
        point.x = static_cast<float>(leave.second.mean_(0));
        point.y = static_cast<float>(leave.second.mean_(1));
        point.z = static_cast<float>(leave.second.mean_(2));
        int ring_idx, sector_idx;
        if (!CalculateSectorId(point, ring_idx, sector_idx))
        {
            continue;
        }
        int z_idx = CalculateLayer(point.z);
        if (z_idx == -1)
        {
            continue;
        }

        double value = leave.second.evals_(2) * leave.second.evals_(0) / (leave.second.evals_(1) * leave.second.evals_(1));
        
        double interval_size = 3 / double(num_intervals);
        if (value < 3) {
            int his_idx = static_cast<int>(std::floor(value / interval_size));
            histogram(0, his_idx)++;
        }

        if (value > 0 && value < 2.4) {
            int shape_id = static_cast<int>(std::floor(value / 0.3));
            grid[ring_idx - 1][sector_idx - 1][z_idx - 1].addShape(shape_id+1);
            double N = 3.0;
            desc_1(ring_idx-1,sector_idx-1) += ((N/2)*(1+log(2*M_PI)) + 0.5*log(cov.determinant())) * z_idx / PC_NUM_Z;
            // desc_1(ring_idx-1,sector_idx-1) += ((N/2)*(1+log(2*M_PI)) + 0.5*log(cov.determinant())) / PC_NUM_Z; // average pooling
        } else {
            continue;
        }
    }

    //     Compute NDTMC matrix using shape weights
    Eigen::MatrixXd desc_2(PC_NUM_RING, PC_NUM_SECTOR);
    desc_2.setZero();
    for (int i = 0; i < PC_NUM_RING; i++)
    {
        for (int j = 0; j < PC_NUM_SECTOR; j++)
        {
            double weight = 0;
            for (int k = 0; k < PC_NUM_Z; k++)
            {
                if (grid[i][j][k].shape_max == -1) {
                    continue;
                }
                weight += double(grid[i][j][k].shape_max * (k + 1)) / PC_NUM_Z;
                // weight += double(grid[i][j][k].shape_max) / PC_NUM_Z; //avarage pooling

            }
            desc_2(i, j) = weight;
        }
    }

    Eigen::MatrixXd desc(2 * PC_NUM_RING, PC_NUM_SECTOR);
    desc << desc_1,
            desc_2;
    return desc;
} // getNDTLeaves_new

float NDTMC::xy2theta(const float &_x, const float &_y)
{
    if ((_x >= 0) & (_y >= 0))
        return static_cast<float>((180 / M_PI) * atan(_y / _x));

    if ((_x < 0) & (_y >= 0))
        return static_cast<float>(180 - ((180 / M_PI) * atan(_y / (-_x))));

    if ((_x < 0) & (_y < 0))
        return static_cast<float>(180 + ((180 / M_PI) * atan(_y / _x)));

    if ((_x >= 0) & (_y < 0))
        return static_cast<float>(360 - ((180 / M_PI) * atan((-_y) / _x)));
    return 0.0;
} // xy2theta

bool NDTMC::CalculateSectorId(pcl::PointXYZI &pt_in, int &ring_idx, int &sctor_idx)
{
    // xyz to ring, sector
    float azim_range = sqrt(pt_in.x * pt_in.x + pt_in.y * pt_in.y);
    float azim_angle = xy2theta(pt_in.x, pt_in.y);
    // if range is out of roi, pass
    if (azim_range > PC_MAX_RADIUS)
        return false;

    ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
    sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);
    return true;
} // CalculateSectorId

// int NDTMC::CalculateLayerWeight(double z) const {
//     // 均分为3段
//     if (z < 0)
//         return 0;
//     int layer = floor(z / PC_LAYER);
//     // int layer = floor(z * 2);
//     if (layer >= PC_MAX_LAYER)
//         return 0;
//     return pow(3, layer);
// } // CalculateLayerWeight

int NDTMC::CalculateLayer(float z) const
{
    if (z < 0 || z > PC_MAX_Z)
    {
        return -1;
    }
    int layer = std::max(std::min(PC_NUM_Z, int(ceil((z / PC_MAX_Z) * PC_NUM_Z))), 1);
    //    int layer = static_cast<int>(std::floor(z / PC_LAYER)) + 1;

    return layer;
}

int NDTMC::GetDistributionWeight(int idx)
{
    NDTCellPtr cell_temp = nullptr;
    cell_temp = &ndtcell[idx];
    // 获取cov的特征向量，转化至局部坐标系，判断方向
    Eigen::Matrix3d cov = cell_temp->cov_;
    // AVP_LOGI << cov << "\n" << idx;
    // Eigen::Quaterniond quaternion(q_w, q_x, q_y, q_z);
    // cov = quaternion * cov * quaternion.conjugate();
    //    cov.normalize();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov); // SelfAdjointEigenSolver计算特征向量和特征值
                                                              //    double value = saes.eigenvalues()(2) * saes.eigenvalues()(0) / (saes.eigenvalues()(1) * saes.eigenvalues()(1));
    double value = -1.0;
    if (saes.eigenvalues()(2) < 0.001 * saes.eigenvalues()(0))
    {
        value = 0.001 * saes.eigenvalues()(0) * saes.eigenvalues()(0) /
                (saes.eigenvalues()(1) * saes.eigenvalues()(1));
    }
    else
    {
        value =
            saes.eigenvalues()(2) * saes.eigenvalues()(0) /
            (saes.eigenvalues()(1) * saes.eigenvalues()(1));
    }
    //    Eigen::Vector3d z_normal(0, 0, 1);
    //    Eigen::Vector3d plane_normal_vector = saes.eigenvectors().col(0);
    //    double angle = acos(plane_normal_vector.dot(z_normal) / (plane_normal_vector.norm())) * 180 / M_PI;
    //    if(!(angle > 70 && angle < 110)){
    //        return -1;
    //    }
    //    if(angle < 10 && angle > -10){
    //        return -1;
    //    }
    //    if (value <= 0.4) {
    //        return 1;
    //    } else if (value > 0.4 && value <= 0.6) {
    //        return 2;
    //    } else if (value > 0.6 && value <= 0.8) {
    //        return 3;
    //    } else if (value > 0.8 && value <= 1.2) {
    //        return 4;
    //    } else if (value > 1.2 && value <= 1.8) {
    //        return 5;
    //    }
    // 存储histogram
    double interval_size = 3 / double(num_intervals);
    if (value < 3)
    {
        int his_idx = static_cast<int>(std::floor(value / interval_size));
        histogram(0, his_idx)++;
    }

    //    if (value <= 0.4) {
    //        return 1;
    //    } else if (value > 0.4 && value <= 0.85) {
    //        return 2;
    //    } else if (value > 0.85 && value <= 1.25) {
    //        return 3;
    //    } else if (value > 1.25 && value <= 2.2) {
    //        return 4;
    //    }
    //    if (value <= 0.3) {
    //        return 1;
    //    } else if (value > 0.3 && value <= 0.6) {
    //        return 2;
    //    } else if (value > 0.6 && value <= 0.9) {
    //        return 3;
    //    } else if (value > 0.9 && value <= 1.2) {
    //        return 4;
    //    } else if (value > 1.2 && value <= 1.5){
    //        return 5;
    //    } else if (value > 1.5 && value <= 1.8){
    //        return 6;
    //    } else if (value > 1.8 && value <= 2.1){
    //        return 7;
    //    } else if (value > 2.1 && value <= 2.4){
    //        return 8;
    //    }
    if (value > 0 && value < 2.4)
    {
        int shape_id = static_cast<int>(std::floor(value / 0.3));
        //        cout << value << " " << shape_id+1 << endl;
        return shape_id + 1;
    }

    return -1;
} // GetDistributionWeight

int NDTMC::CalculateShapeWeight(const GridCell &cell)
{
    constexpr int kPCMaxLayer = 1;
    Eigen::Map<const Eigen::Array<int, kPCMaxLayer, 1>> shapes(&cell.shape_max);
    cout << "shapes : " << shapes << endl;
    Eigen::Array<int, kPCMaxLayer, 1> max_shapes = shapes.max(Eigen::Array<int, kPCMaxLayer, 1>::Constant(-1));
    cout << "max_shapes : " << max_shapes << endl;
    Eigen::Array<double, kPCMaxLayer, 1> max_layer_range = Eigen::Array<int, kPCMaxLayer, 1>::LinSpaced(kPCMaxLayer, 0,
                                                                                                        kPCMaxLayer -
                                                                                                            1)
                                                               .cast<double>();
    cout << "max_layer_range : " << max_layer_range << endl;
    Eigen::Array<double, kPCMaxLayer, 1> weights = max_shapes.cast<double>() * Eigen::pow(9.0, max_layer_range);
    cout << "weights : " << weights << endl;
    int weight = weights.sum();
    return weight > 0 ? weight : 0;
}

Eigen::MatrixXd NDTMC::createNDTMC_KITTI()
{
    //    double gridStep = 0.75;
    //    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    //    voxel_filter.setInputCloud(pc_in);
    //    voxel_filter.setLeafSize(gridStep, gridStep, gridStep);
    //    voxel_filter.filter(*pc_in);
    GridBin grid[PC_NUM_RING][PC_NUM_SECTOR][PC_NUM_Z];
    for (int idx = 0; idx < static_cast<int>(pc_in->size()); ++idx)
    {
        pcl::PointXYZI pt = pc_in->points[idx];
        // 1. 根据xy的值计算它属于哪一个bin
        int ring_idx, sector_idx;
        if (!CalculateSectorId(pt, ring_idx, sector_idx))
        {
            continue;
        }
        // 2. 根据z值计算它所在的层（不同层具有不同的权重）
        int z_idx = CalculateLayer(pt.z);
        if (z_idx == -1)
        {
            continue;
        }
        // 3. 添加点到对应的grid中
        grid[ring_idx - 1][sector_idx - 1][z_idx - 1].addPoint(pt.x, pt.y, pt.z);
    }
    // 4. 计算每个bin中的g值，作为描述子
    Eigen::MatrixXd desc(PC_NUM_RING, PC_NUM_SECTOR);
    //    Eigen::Tensor<double, 3> desc_3d(PC_NUM_RING, PC_NUM_SECTOR, PC_MAX_LAYER);
    //    desc_3d.setZero();
    desc.setZero();
    ofstream ff;
    //    ff.open("/home/liaolizhou/Myproject_copy/data/cov.txt", ios::out);
    for (int k = 0; k < PC_NUM_Z; ++k)
    {
        //        desc.setZero();
        for (int i = 0; i < PC_NUM_RING; ++i)
        {
            for (int j = 0; j < PC_NUM_SECTOR; ++j)
            {
                if (grid[i][j][k].num_of_points < 3)
                    continue;
                Eigen::Matrix3d cov = grid[i][j][k].cov;
                //                cov.normalize();
                //                cout << cov.norm() << endl;
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov); // SelfAdjointEigenSolver计算特征向量和特征值
                double value = -1.0;
                if (saes.eigenvalues()(2) < 0.001 * saes.eigenvalues()(0))
                {
                    value = 0.001 * saes.eigenvalues()(0) * saes.eigenvalues()(0) /
                            (saes.eigenvalues()(1) * saes.eigenvalues()(1));
                }
                else
                {
                    value =
                        saes.eigenvalues()(2) * saes.eigenvalues()(0) /
                        (saes.eigenvalues()(1) * saes.eigenvalues()(1));
                }
                if (cov.norm() > 10)
                    continue;
                //                value =
                //                        saes.eigenvalues()(2) * saes.eigenvalues()(0) /
                //                        (saes.eigenvalues()(1) * saes.eigenvalues()(1));
                //                desc_3d(i, j, k) = value;
                //                ff << "mean:" << endl;
                //                ff << grid[i][j][k].mean(0) << " " << grid[i][j][k].mean(1) << " " << grid[i][j][k].mean(2) << endl;
                //                ff << "cov:" << endl;
                //                ff << cov << endl;
                //                if (value <= 0.4) {
                //                    desc(i, j) += pow(5.0, k) * 1;
                //                } else if (value > 0.4 && value <= 0.85) {
                //                    desc(i, j) += pow(5.0, k) * 2;
                //                } else if (value > 0.85 && value <= 1.25) {
                //                    desc(i, j) += pow(5.0, k) * 3;
                //                } else if (value > 1.25 && value <= 2.2) {
                //                    desc(i, j) += pow(5.0, k) * 4;
                //                }
                //                cout << cov.norm() << endl;
                //                int cov_norm = cov.norm() > 5 ? 5 : cov.norm();
                //                desc(i, j) += value * cov_norm * (k+1);
                double N = 4.0;
                desc(i, j) += (N / 2) * (1 + log(2 * M_PI)) + 0.5 * log(cov.determinant());
            }
        }
    }
    //    ff.close();
    //    for (int i = 0; i < desc.cols(); i++) {
    //        desc.col(i).normalize();
    //    }
    return desc;
}

NDTMC::NDTMC()
{
}

Eigen::MatrixXd NDTMC::createNDTMC()
{
    transformToNDTForm(resolution);
    return getNDTLeaves_new(voxel_grid_frame.getLeaves());
}

Eigen::MatrixXd NDTMC::createNDTMC(const std::map<std::size_t, NDTMC::Leaf> &leaves)
{
    return getNDTLeaves_new(leaves);
}
