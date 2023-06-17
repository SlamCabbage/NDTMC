import os
from multiprocessing import Process

def save_desc(folder_name):
    input_folder = f"/media/liaolizhou/Dataset/NIO_Underground/{folder_name}"
    output_folder = f"/media/liaolizhou/Dataset/NIO_Underground/{folder_name}"
    for root, dirs, files in os.walk(input_folder):
        for dir_name in dirs:
            if "database" in dir_name:
                input_path = os.path.join(root, dir_name+'/submaps_pcd')
                output_path = os.path.join(output_folder, dir_name+'/submaps_bin')

                print("input folder: ", input_path)
                print("output folder: ", output_path)
                if not os.path.exists(output_path):
                    # 如果不存在则创建文件夹
                    os.makedirs(output_path)
                # print(f"./../build/run_demo {input_path} {output_path}")
                os.system(f"./../build/run_demo {input_path} {output_path}")

if __name__ == '__main__':
    folder_names = ['Lixiangguoji', 'Rongke', 'YingwangCenter', 'Yinzuo']
    # folder_names = ['07']
    processes = []
    for folder_name in folder_names:
        process = Process(target=save_desc, args=(folder_name,))
        process.start()
        processes.append(process)
    for process in processes:
        process.join()