import os
from multiprocessing import Process

def process_folder(folder_name, testdatan):
    database = f"/media/liaolizhou/Dataset/NIO_Underground/{folder_name}/database"
    testdata = f"/media/liaolizhou/Dataset/NIO_Underground/{folder_name}/{testdatan}"
    print(f"./../build/run_demo {folder_name} {database} {testdata}")
    os.system(f"./../build/run_demo {folder_name} {database} {testdata}")

if __name__ == '__main__':
    # folder_names = ['Lixiangguoji', 'Rongke', 'YingwangCenter', 'Yinzuo']
    folder_names = ['Lixiangguoji', 'YingwangCenter', 'Yinzuo']
    testdata = ['testdata1', 'testdata2']
    # testdata = ['testdata2']
    # folder_names = ['Lixiangguoji']
    # folder_names = ['07']
    for testdatan in testdata:
        processes = []
        for folder_name in folder_names:
            process = Process(target=process_folder, args=(folder_name,testdatan,))
            process.start()
            processes.append(process)
        for process in processes:
            process.join()
