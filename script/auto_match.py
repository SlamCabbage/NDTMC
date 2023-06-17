import os
from multiprocessing import Process

def process_folder(folder_name):
    input_folder = f"/media/liaolizhou/Dataset/KITTI/{folder_name}/desc_1m_average"
    output_txt = f"/media/liaolizhou/Dataset/KITTI/{folder_name}/results_1m_average.txt"
    print(f"./../build/run_demo {input_folder} {output_txt}")
    os.system(f"./../build/run_demo {input_folder} {output_txt} {folder_name}")

if __name__ == '__main__':
    folder_names = ['00', '02', '05', '06', '07', '08']
    # folder_names = ['07']
    processes = []
    for folder_name in folder_names:
        process = Process(target=process_folder, args=(folder_name,))
        process.start()
        processes.append(process)
    for process in processes:
        process.join()
    #     input_folder = f"/media/liaolizhou/Dataset/KITTI/{folder_name}/desc"
    #     output_txt = f"/media/liaolizhou/Dataset/KITTI/{folder_name}/results_2.txt"
    #     print(f"./../cmake-build-debug/run_demo {input_folder} {output_txt}")
    #     os.system(f"./../cmake-build-debug/run_demo {input_folder} {output_txt}")
