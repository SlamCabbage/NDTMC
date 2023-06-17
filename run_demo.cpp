//
// Created by liaolizhou on 23-4-3.
//

#include <utility.h>
#include "pcl_voxel_covariance.hpp"

using namespace std;

enum DataType
{
    KITTI,
    NIO,
    PCD2BIN,
    NIO_database
};

void loadNDTMap(const std::string &file_name, MyVoxelGridCovariance& myvg)
{
    std::ifstream ifs(file_name, std::ios::binary);
    if (!ifs)
    {
        PCL_ERROR("open ---->%s fail!!!\n", file_name.c_str());
        return;
    }
    // save leaf size
    float leaf_size[4];
    ifs.read((char *)leaf_size, sizeof(float) * 4);
    Eigen::Vector4f leaf_size_vec = Eigen::Map<Eigen::Vector4f>(leaf_size);
    PCL_INFO("load leaf size is %.2f %.2f %.2f %.2f\n", leaf_size_vec[0], leaf_size_vec[1], leaf_size_vec[2],
             leaf_size_vec[3]);
    myvg.setLeafSize(leaf_size_vec);
    myvg.loadVoxelGrid(ifs);
    ifs.close();
    return;
} // loadNDTMap

void matchForNIO(char **argv)
{
    const string name = argv[1];
    const string database_path = argv[2];
    const string testdata_path = argv[3];
    const string result_file = testdata_path + "/results_1m_desc_0601_1.txt";
    ofstream ofss(result_file, ios::out);
    ofss << "desc cosine_distance 0-3.0/0.3 k+1/pc_num_Z \t" << "40 180 3" << endl; 
    // 1. Read database ndt map
    MyVoxelGridCovariance myvg;
    const string ndt_map = database_path + "/map.bin";
    loadNDTMap(ndt_map, myvg);
    // ofss << "load NDT Map success" << endl;
    cout << "load NDT Map success" << endl;
    const string poses_database = database_path + "/graph.txt";
    vector<vector<string>> Pose = readPoseTxt(poses_database);
    // ofss << "read database pose txt success" << endl;
    cout << "read database pose txt success" << endl;
    // 读取所有的database leaves
    // ofss << name << " : start getting submaps from ndt map" << endl;
    cout << name << " : start getting submaps from ndt map" << endl;
    vector<map<size_t, NDTMC::Leaf>> database_leaves;
    thread_read_ndtsubmap(Pose, database_leaves, myvg);
    // ofss << name << " : get submaps from ndt map success" << endl;
    cout << name << " : get submaps from ndt map success" << endl;
    // 2. 遍历testdata
    const string poses_loop = testdata_path + "/graphloop.txt";
    vector<vector<string>> Poseloop = readPoseTxt(poses_loop);
    cout << "success read Poseloop" << endl;
    // const string testdata_bin_path = testdata_path + "/submaps_bin/";
    const string testdata_bin_path = testdata_path + "/submaps_bin_new/1m/";
    int real = 0;
    int all_loop_num = 0;
    vector<int> topn(10);
    for(int i = 0; i < Poseloop.size(); ++i){
        int frame = std::stoi(Poseloop[i][0]);
        int hasloop = std::stoi(Poseloop[i][8]);
        if (hasloop != 1) continue;
        ofss << frame << endl;
        all_loop_num ++;
        // 3. 读取testdata当前帧的描述子
        const string frame_path = testdata_bin_path + to_string(frame) + ".bin";
        NDTMC ndtsc_manager_testdata(1);
        auto leaves = ndtsc_manager_testdata.readNIOBin(frame_path);
        Eigen::MatrixXd testdata = ndtsc_manager_testdata.createNDTMC(leaves);
        vector<NDTMC::LoopResults> all_results;
        // 4. 遍历database描述子
        thread_run(Pose, database_leaves, testdata, all_results);

        if (all_results.size() >= 2)
        {
            sort(all_results.begin(), all_results.end(), compareFunction);
        }
        
        cout << " " << name << " " << frame << " " << double(i)/double(Poseloop.size()) << endl;
        for (int k = 0; k < all_results.size(); ++k)
        {
            if (k >= 10)
                break;
            NDTMC::LoopResults p = all_results[k];
            Eigen::Matrix4d pose = getPose(p.frame_id - stoi(Pose[0][0]), Pose);
            double tx = stod(Poseloop[i][1]);
            double ty = stod(Poseloop[i][2]);
            double tz = stod(Poseloop[i][3]);
            double tx_ = stod(Pose[p.frame_id-stoi(Pose[0][0])][1]);
            double ty_ = stod(Pose[p.frame_id-stoi(Pose[0][0])][2]);
            double tz_ = stod(Pose[p.frame_id-stoi(Pose[0][0])][3]);
            double dis = sqrt((tx-tx_)*(tx-tx_) + (ty-ty_)*(ty-ty_) + (tz-tz_)*(tz-tz_));
            ofss << p.frame_id << " " << p.similarity << " " << p.shift << " " << dis << std::endl;
            cout << k << " " << frame << " " << p.frame_id << " " << p.similarity << " " << p.shift << " " << dis << std::endl;
            if(dis < 4){
                real++;
                for(int l = k; l < topn.size(); ++l){
                    topn[l]++;
                }
                break;
            }
        }
    }
    for(int m = 0; m < topn.size(); ++m){
        ofss << "top " << m << " : " << double(topn[m])/double(all_loop_num) << endl;
    }
    ofss.close();
}

void matchForKitti(char **argv)
{
    const char *matrix_path = argv[1];
    const string results_path = argv[2];
    const string datatset_name = argv[3];
    string matrix_path_string = matrix_path;
    vector<string> frame_name = read_file_name(matrix_path);
    vector<int> frames;
    vector<string> frame_name_copy;

    for (const auto &frame_ : frame_name)
    {
        int frame = stoi(frame_.substr(0, frame_.find_last_of('.')));
    }
    sort(frames.begin(), frames.end());
    sort(frame_name.begin(), frame_name.end());
    // 读取所有的desc
    vector<Eigen::MatrixXd> all_desc;
    vector<Eigen::VectorXd> all_hist;
    for (const auto &frame : frame_name)
    {
        string desc_path = matrix_path_string + '/' + frame;
        cout << "start load " << desc_path << endl;
        auto matrix = read_matrix(desc_path);
        Eigen::MatrixXd query = matrix.first;
        Eigen::MatrixXd histogram = matrix.second;
        all_desc.emplace_back(query);
        all_hist.emplace_back(histogram);
    }

    ofstream ofss(results_path, ios::out);
    for (int i = 50; i < all_desc.size(); ++i)
    {
        cout << datatset_name << " : " << i << " " << all_desc.size() - 1 << " " << (double(i) / double(all_desc.size() - 1)) * 100 << " %" << endl;
        ofss << i << std::endl;
        Eigen::MatrixXd query = all_desc[i];
        Eigen::VectorXd query_hist = all_hist[i];
        vector<NDTMC::LoopResults> all_results;
        parallelLoop(all_desc, query, all_results, i);
        if (all_results.size() >= 2)
        {
            sort(all_results.begin(), all_results.end(), compareFunction);
        }

        for (int k = 0; k < all_results.size(); ++k)
        {
            if (k >= 10)
                break;
            NDTMC::LoopResults p = all_results[k];
            Eigen::VectorXd candidate_hist = all_hist[p.frame_id];
            double ans = query_hist.dot(candidate_hist) / (query_hist.norm() * candidate_hist.norm());
            double ans1 = (query_hist - candidate_hist).norm();
            ofss << p.frame_id << " " << p.similarity << " " << p.shift << " " << ans << " " << ans1 << std::endl;
            //    cout << p.frame_id << " " << p.similarity << " " << p.shift  << std::endl;
        }
    }
    ofss.close();
    return;
}

void saveDesc(char **argv)
{
    const string data_type = argv[1];
    const char *path = argv[2];
    const string save_dir = argv[3];
    DataType type;
    if (data_type == "KITTI")
    {
        type = KITTI;
    }
    else if (data_type == "NIO")
    {
        type = NIO;
    }
    else if (data_type == "PCD2BIN")
    {
        type = PCD2BIN;
    }
    else
    {
        // 处理无法识别的数据类型
        cerr << "the input data_type is wrong" << endl;
        return;
    }

    vector<string> bin_name = read_file_name(path);
    string bin_path(path);

    int num_threads = 8; // 线程数
    int num_files = bin_name.size();
    int num_files_per_thread = num_files / num_threads;

    // 创建线程
    vector<thread> threads;
    for (int i = 0; i < num_threads; i++)
    {
        int start_idx = i * num_files_per_thread;
        int end_idx = (i == num_threads - 1) ? num_files - 1 : start_idx + num_files_per_thread - 1;
        vector<string> file_list(bin_name.begin() + start_idx, bin_name.begin() + end_idx + 1);

        switch (type)
        {
        case KITTI:
            threads.emplace_back(processFiles, file_list, bin_path, save_dir);
            break;
        case NIO:
            threads.emplace_back(processFiles_PCD, file_list, bin_path, save_dir);
            break;
        case PCD2BIN:
            threads.emplace_back(savePCDFilesToBin, file_list, bin_path, save_dir);
            break;
        }
    }
    // 等待线程完成
    for (auto &t : threads)
    {
        if (t.joinable())
            t.join();
    }
    return;
}

int main(int argc, char **argv)
{
    // ****************************************************
    //                 save descriptor
    // ****************************************************
    // saveDesc(argv);
    // ****************************************************

    // ****************************************************
    //                    KITTI test
    // ****************************************************
    matchForKitti(argv);
    // ****************************************************

    // ****************************************************
    //                 NIO_dataset test
    // ****************************************************
    // matchForNIO(argv);
    // ****************************************************

    return 0;
}
