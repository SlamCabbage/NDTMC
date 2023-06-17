import os
from multiprocessing import Process

def save_desc(folder_name):
    input_folder = f"/media/liaolizhou/Dataset/KITTI/{folder_name}/velodyne"
    output_folder = f"/media/liaolizhou/Dataset/KITTI/{folder_name}/desc_1m_average"
    data_type = "KITTI"
    if not os.path.exists(output_folder):
        # 如果不存在则创建文件夹
        os.makedirs(output_folder)
    print(f"./../build/run_demo data_type {input_folder} {output_folder}")
    os.system(f"./../build/run_demo data_type {input_folder} {output_folder}")

if __name__ == '__main__':
    folder_names = ['00', '02', '05', '06', '07', '08']
    # folder_names = ['07']
    processes = []
    for folder_name in folder_names:
        process = Process(target=save_desc, args=(folder_name,))
        process.start()
        processes.append(process)
    for process in processes:
        process.join()