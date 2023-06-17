#pragma once
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <NDTMC.h>
#include <dirent.h>
#include <algorithm>
#include <pcl_voxel_covariance.hpp>

using namespace std;

void testGridBin()
{
    // Seed the random number generator
    srand(0);

    // Test for n = 3 to 10
    for (int n = 3; n <= 10; ++n)
    {
        // Create a new GridBin object
        NDTMC::GridBin bin;

        // Add n random points
        for (int i = 0; i < n; ++i)
        {
            float x = static_cast<float>(rand()) / RAND_MAX;
            float y = static_cast<float>(rand()) / RAND_MAX;
            float z = static_cast<float>(rand()) / RAND_MAX;
            //            cout << "x: " << x << " y: " << y << " z: " << z << endl;
            bin.addPoint(x, y, z);
        }

        // Output the results
        std::cout << "n = " << n << std::endl;
        std::cout << "Points: " << std::endl
                  << bin.Points << std::endl;
        std::cout << "Mean: " << std::endl
                  << bin.mean << std::endl;
        std::cout << "Eigen Mean: " << std::endl
                  << bin.Points.colwise().mean() << std::endl;
        std::cout << "Covariance: " << std::endl
                  << bin.cov << std::endl;

        Eigen::MatrixXd points = bin.Points;
        Eigen::MatrixXd cov(points.rows(), points.cols());
        for (size_t i = 0; i < cov.rows(); i++)
        {
            cov.row(i) = points.row(i) - bin.Points.colwise().mean();
        }
        cov = cov.transpose() * cov / (cov.rows() - 1);
        std::cout << "Eigen Covariance: " << std::endl
                  << cov << std::endl;
    }
}

pair<Eigen::MatrixXd, Eigen::MatrixXd> read_matrix(const string &matrix_path)
{
    Eigen::MatrixXd M;
    Eigen::VectorXd H;
    std::ifstream fin(matrix_path);
    if (fin)
    {
        int rows, cols;
        fin >> rows >> cols;
        M.resize(rows, cols);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                fin >> M(i, j);
            }
        }
        fin >> rows >> cols;
        H.resize(cols);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                fin >> H(j);
            }
        }
    }
    else
    {
        std::cerr << "Failed to open file!" << std::endl;
    }
    return {M, H};
}

vector<string> read_file_name(const char *path)
{
    vector<string> bin_name;
    DIR *dir = opendir(path);
    if (dir)
    {
        dirent *dp;
        while ((dp = readdir(dir)) != nullptr)
        {
            if (dp->d_type == DT_REG)
            { // 只输出普通文件
                bin_name.emplace_back(dp->d_name);
            }
        }
        closedir(dir);
    }
    return bin_name;
}

// 处理PCD点云的函数
void savePCDFilesToBin(const vector<string> &file_list, const string &path, const string &save_dir)
{
    for (const auto &bin_name : file_list)
    {
        // Define the path of the .bin
        string bin_path = path + "/" + bin_name;
        std::string frame = bin_name.substr(0, bin_name.find_last_of('.'));

        string save_bin = save_dir + "/" + frame + ".bin";
        cout << "start processing " << frame << "-th frame" << endl;
        NDTMC ndtsc_manager(1.0);
        ndtsc_manager.readPCD(bin_path);
        ndtsc_manager.transformToNDTForm(1.0);

        std::ofstream outfile(save_bin, std::ios::binary);
        if (!outfile)
        {
            std::cerr << "Failed to open file: " << save_bin << std::endl;
            return;
        }
        auto leaves_ = ndtsc_manager.voxel_grid_frame.getLeaves();
        for (const auto &[key, leaf] : leaves_)
        {
            outfile.write(reinterpret_cast<const char *>(&leaf.nr_points), sizeof(int));
            outfile.write(reinterpret_cast<const char *>(&leaf.mean_), sizeof(Eigen::Vector3d));
            outfile.write(reinterpret_cast<const char *>(&leaf.cov_), sizeof(Eigen::Matrix3d));
            outfile.write(reinterpret_cast<const char *>(&leaf.icov_), sizeof(Eigen::Matrix3d));
            outfile.write(reinterpret_cast<const char *>(&leaf.evecs_), sizeof(Eigen::Matrix3d));
            outfile.write(reinterpret_cast<const char *>(&leaf.evals_), sizeof(Eigen::Vector3d));
        }
        outfile.close();
    }
}

void processFiles_PCD(const vector<string> &file_list, const string &path, const string &save_dir)
{
    for (const auto &bin_name : file_list)
    {
        string bin_path = path + "/" + bin_name;
        std::string frame = bin_name.substr(0, bin_name.find_last_of('.'));

        string save_desc = save_dir + "/" + frame + ".txt";
        ofstream fout(save_desc, ios::out);

        cout << "start processing " << frame << "-th frame" << endl;
        NDTMC ndtsc_manager(1.0);
        ndtsc_manager.readPCD(bin_path);
        fout << 2 * ndtsc_manager.PC_NUM_RING << " " << ndtsc_manager.PC_NUM_SECTOR << endl;
        fout << ndtsc_manager.createNDTMC() << endl;
        //        ndtsc_manager.resolution = 2.0;
        //        ndtsc_manager.readKittiBin(bin_path);
        //        fout << ndtsc_manager.createNDTMC() << endl;
        fout << 1 << " " << ndtsc_manager.num_intervals << endl;
        fout << ndtsc_manager.histogram << endl;
        fout.close();
    }
}

// 处理一组文件的函数
void processFiles(const vector<string> &file_list, const string &path, const string &save_dir)
{
    for (const auto &bin_name : file_list)
    {
        // Define the path of the .bin
        string bin_path = path + "/" + bin_name;
        std::string frame = bin_name.substr(0, bin_name.find_last_of('.'));

        // Define the descriptor storage path   
        string save_desc = save_dir + "/" + frame + ".txt";
        ofstream fout(save_desc, ios::out);

        // creat ndtmc descriptor
        cout << "start processing " << frame << "-th frame" << endl;
        NDTMC ndtsc_manager(1.0);
        ndtsc_manager.readKittiBin(bin_path);
        fout << 2 * ndtsc_manager.PC_NUM_RING << " " << ndtsc_manager.PC_NUM_SECTOR << endl;
        fout << ndtsc_manager.createNDTMC() << endl;
        //        ndtsc_manager.resolution = 2.0;
        //        ndtsc_manager.readKittiBin(bin_path);
        //        fout << ndtsc_manager.createNDTMC() << endl;
        fout << 1 << " " << ndtsc_manager.num_intervals << endl;
        fout << ndtsc_manager.histogram << endl;
        fout.close();
    }
}

bool compareFunction(const NDTMC::LoopResults &a, const NDTMC::LoopResults &b)
{
    return a.similarity < b.similarity;
}

// Define a function that each thread will execute
std::mutex mtx;

void threadFunction(const std::vector<Eigen::MatrixXd> &all_desc,
                    Eigen::MatrixXd &query,
                    std::vector<NDTMC::LoopResults> &all_results,
                    int start_idx, int end_idx)
{
    for (int j = start_idx; j < end_idx; ++j)
    {
        Eigen::MatrixXd candidate = all_desc[j];
        vector<pair<double, int>> results = NDTMC::distanceBtnNDTScanContext(query, candidate);
        const auto p = std::min_element(results.begin(), results.end());
        NDTMC::LoopResults scv(p->first, p->second, j);

        // Lock the mutex before modifying all_results
        std::lock_guard<std::mutex> lock(mtx);
        all_results.emplace_back(scv);
    }
}

std::mutex mtx2;
void threadFunction_loopwithinHist(const std::vector<Eigen::MatrixXd> &all_desc,
                                   Eigen::MatrixXd &query,
                                   std::vector<NDTMC::LoopResults> &all_results,
                                   std::vector<pair<double, int>> &all_hist_results,
                                   int start_idx, int end_idx)
{
    for (int j = start_idx; j < end_idx; ++j)
    {
        Eigen::MatrixXd candidate = all_desc[all_hist_results[j].second];
        vector<pair<double, int>> results = NDTMC::distanceBtnNDTScanContext(query, candidate);
        const auto p = std::min_element(results.begin(), results.end());
        NDTMC::LoopResults scv(p->first, p->second, all_hist_results[j].second);
        // Lock the mutex before modifying all_results
        std::lock_guard<std::mutex> lock(mtx2);
        all_results.emplace_back(scv);
    }
}

std::mutex mtx1;
void threadFunction_hist(const std::vector<Eigen::VectorXd> &all_hist,
                         Eigen::VectorXd &query_hist,
                         std::vector<pair<double, int>> &all_hist_results,
                         int start_idx, int end_idx)
{
    for (int j = start_idx; j < end_idx; ++j)
    {
        Eigen::VectorXd candidate_hist = all_hist[j];
        double ans = query_hist.dot(candidate_hist) / (query_hist.norm() * candidate_hist.norm());
        // Lock the mutex before modifying all_results
        std::lock_guard<std::mutex> lock(mtx1);
        all_hist_results.emplace_back(ans, j);
    }
}

void parallelHist(const std::vector<Eigen::VectorXd> &all_hist,
                  Eigen::VectorXd &query_hist,
                  vector<pair<double, int>> &all_hist_results,
                  int idx)
{
    const int num_threads = 8;
    if (idx - 49 < num_threads)
    {
        for (int j = 0; j < idx - 49; ++j)
        {
            Eigen::VectorXd candidate_hist = all_hist[j];
            double ans = query_hist.dot(candidate_hist) / (query_hist.norm() * candidate_hist.norm());
            all_hist_results.emplace_back(ans, j);
        }
        return;
    }
    // Determine the number of iterations to perform per thread
    const int num_iterations = (idx - 49) / num_threads;

    // Launch the threads
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i)
    {
        const int start_idx = i * num_iterations;
        const int end_idx = (i == num_threads - 1) ? idx - 49 : start_idx + num_iterations;
        threads.emplace_back(threadFunction_hist, std::ref(all_hist), std::ref(query_hist), std::ref(all_hist_results),
                             start_idx, end_idx);
    }

    // Wait for the threads to finish
    for (auto &t : threads)
    {
        t.join();
    }
}

// Define a function to launch the threads
void parallelLoop(const std::vector<Eigen::MatrixXd> &all_desc,
                  Eigen::MatrixXd &query,
                  std::vector<NDTMC::LoopResults> &all_results,
                  int idx)
{
    const int num_threads = 8;
    if (idx - 49 < num_threads)
    {
        for (int j = 0; j < idx - 49; ++j)
        {
            Eigen::MatrixXd candidate = all_desc[j];
            vector<pair<double, int>> results = NDTMC::distanceBtnNDTScanContext(query, candidate);
            const auto p = std::min_element(results.begin(), results.end());
            NDTMC::LoopResults scv(p->first, p->second, j);
            all_results.emplace_back(scv);
        }
        return;
    }
    // Determine the number of iterations to perform per thread
    const int num_iterations = (idx - 49) / num_threads;

    // Launch the threads
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i)
    {
        const int start_idx = i * num_iterations;
        const int end_idx = (i == num_threads - 1) ? idx - 49 : start_idx + num_iterations;
        threads.emplace_back(threadFunction, std::ref(all_desc), std::ref(query), std::ref(all_results), start_idx,
                             end_idx);
    }

    // Wait for the threads to finish
    for (auto &t : threads)
    {
        t.join();
    }
}

// Define a function to launch the threads
void histLoop(const std::vector<Eigen::MatrixXd> &all_desc,
              Eigen::MatrixXd &query,
              std::vector<NDTMC::LoopResults> &all_results,
              vector<pair<double, int>> &all_hist_results)
{
    int size = all_hist_results.size() >= 20 ? 20 : static_cast<int>(all_hist_results.size());
    for (int j = 0; j < size; ++j)
    {
        Eigen::MatrixXd candidate = all_desc[all_hist_results[j].second];
        vector<pair<double, int>> results = NDTMC::distanceBtnNDTScanContext(query, candidate);
        const auto p = std::min_element(results.begin(), results.end());
        NDTMC::LoopResults scv(p->first, p->second, all_hist_results[j].second);
        all_results.emplace_back(scv);
    }
}

void LoopwithinHist(const std::vector<Eigen::MatrixXd> &all_desc,
                    Eigen::MatrixXd &query,
                    std::vector<NDTMC::LoopResults> &all_results,
                    vector<pair<double, int>> &all_hist_results)
{
    // int size = all_hist_results.size() >= 20 ? 20 : static_cast<int>(all_hist_results.size());
    int size;
    for (size = 0; size < all_hist_results.size(); ++size)
    {
        if (all_hist_results[size].first > 120 || size >= 50)
            break;
    }
    // auto it = upper_bound(all_hist_results.begin(), all_hist_results.end(), 120, cmp);

    // if (it == all_hist_results.end())
    // {
    //     size = all_hist_results.size();
    // }
    // else
    // {
    //     int index = std::distance(all_hist_results.begin(), it);
    //     size = index;
    // }

    const int num_threads = 8;
    if (size < num_threads)
    {
        for (int j = 0; j < size; ++j)
        {
            Eigen::MatrixXd candidate = all_desc[all_hist_results[j].second];
            vector<pair<double, int>> results = NDTMC::distanceBtnNDTScanContext(query, candidate);
            const auto p = std::min_element(results.begin(), results.end());
            NDTMC::LoopResults scv(p->first, p->second, all_hist_results[j].second);
            all_results.emplace_back(scv);
        }
        return;
    }
    // Determine the number of iterations to perform per thread
    const int num_iterations = size / num_threads;

    // Launch the threads
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i)
    {
        const int start_idx = i * num_iterations;
        const int end_idx = (i == num_threads - 1) ? size : start_idx + num_iterations;
        threads.emplace_back(threadFunction_loopwithinHist, std::ref(all_desc), std::ref(query), std::ref(all_results), std::ref(all_hist_results), start_idx,
                             end_idx);
    }

    // Wait for the threads to finish
    for (auto &t : threads)
    {
        t.join();
    }
}

bool split(const std::string &str, const char patter, std::vector<std::string> &res)
{
    std::stringstream input(str);
    std::string temp;
    int i = 0;
    while (getline(input, temp, patter))
    {
        if (i == 0 && temp != "VERTEX_SE3:QUAT")
            return false;
        if (i < 10 && i > 0)
        {
            res.push_back(temp);
        }
        i++;
    }
    return true;
}

std::vector<std::vector<std::string>> readPoseTxt(std::string file)
{
    std::vector<std::vector<std::string>> Pose;
    std::ifstream infile;
    infile.open(file.data()); // 将⽂件流对象与⽂件连接起来
    assert(infile.is_open()); // 若失败,则输出错误消息,并终⽌程序运⾏
    std::string s;
    while (getline(infile, s))
    {
        std::vector<std::string> temp;
        if (split(s, ' ', temp))
        {
            Pose.emplace_back(temp);
        }
    }
    infile.close(); // 关闭⽂件输⼊流
    return Pose;
}

Eigen::Matrix4d getPose(int frame, std::vector<std::vector<std::string>> &Pose)
{
    double t_x = std::stod(Pose[frame][1]);
    double t_y = std::stod(Pose[frame][2]);
    double t_z = std::stod(Pose[frame][3]);
    double q_x = std::stod(Pose[frame][4]);
    double q_y = std::stod(Pose[frame][5]);
    double q_z = std::stod(Pose[frame][6]);
    double q_w = std::stod(Pose[frame][7]);
    Eigen::Vector3d location(t_x, t_y, t_z);
    Eigen::Quaterniond quaternion1(q_w, q_x, q_y, q_z);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = quaternion1.toRotationMatrix();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = rotation_matrix;
    pose.block<3, 1>(0, 3) = location;
    return pose;
}

std::mutex mtx3;
void thread_run(vector<vector<string>> &Pose, vector<map<size_t, NDTMC::Leaf>> &database_leaves, Eigen::MatrixXd &testdata, vector<NDTMC::LoopResults> &all_results)
{
    vector<thread> threads;
    const int num_of_threads = 8;
    int num_poses = Pose.size();
    int poses_per_thread = num_poses / num_of_threads;
    for (int i = 0; i < num_of_threads; ++i)
    {
        int start = i * poses_per_thread;
        int end = (i == num_of_threads-1) ? num_poses : (i + 1) * poses_per_thread;
        threads.emplace_back([start, end, &Pose, &database_leaves, &testdata, &all_results]()
                             {
            for (int j = start; j < end; ++j) {
                int frame_database = stoi(Pose[j][0]);
                // cout << "start processing : " << frame_database << " " << Pose.size() << endl;
                auto leaves = database_leaves[j];
                NDTMC ndtsc_manager(1);
                Eigen::MatrixXd database = ndtsc_manager.createNDTMC(leaves);
                vector<pair<double, int>> results = NDTMC::distanceBtnNDTScanContext(testdata, database);
                std::sort(results.begin(), results.end(), [](pair<double, int>& a, pair<double, int>& b){return a.first < b.first;});
                auto p = results[0];
                // cout << p.first << " " <<  p.second << " " <<  frame_database << endl;
                NDTMC::LoopResults scv(p.first, p.second, frame_database);
                std::lock_guard<std::mutex> lock(mtx3);
                all_results.emplace_back(scv);
            } });
    }

    // Wait for all threads to finish
    for (auto &thread : threads)
    {
        thread.join();
    }
}

void thread_read_ndtsubmap(vector<vector<string>> &Pose, vector<map<size_t, NDTMC::Leaf>> &database_leaves, MyVoxelGridCovariance &myvg)
{
    // Start 8 threads to process the loop in parallel
    vector<thread> threads;
    const int num_of_threads = 8;
    int num_poses = Pose.size();
    int poses_per_thread = num_poses / num_of_threads;
    vector<map<size_t, NDTMC::Leaf>> all_leaves(num_poses);
    for (int i = 0; i < num_of_threads; ++i)
    {
        int start = i * poses_per_thread;
        int end = (i == num_of_threads-1) ? num_poses : (i + 1) * poses_per_thread;
        threads.emplace_back([start, end, &Pose, &myvg, &all_leaves]()
                             {
        for (int j = start; j < end; ++j) {
            // cout << "processing database " << j << "-th " << "frame's leaves" << endl;
            int frame_database = stoi(Pose[j][0]);
            Eigen::Matrix4d pose = getPose(frame_database - stoi(Pose[0][0]), Pose);
            auto leaves = myvg.getNDTSubMap(pose, 80);
            all_leaves[j] = leaves;
        } });
    }

    // Wait for all threads to finish
    for (auto &thread : threads)
    {
        thread.join();
    }

    // Copy the results from all_leaves to database_leaves
    database_leaves.resize(num_poses);
    for (int i = 0; i < num_poses; ++i)
    {
        database_leaves[i] = std::move(all_leaves[i]);
    }
}