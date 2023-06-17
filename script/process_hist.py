import numpy as np

poses = []
with open('/media/liaolizhou/Dataset/KITTI/06/poses.txt', 'r') as f:
    for line in f:
        numbers = line.strip().split()
        t_x, t_y, t_z = float(numbers[3]), float(numbers[7]), float(numbers[11])
        pose = np.array([t_x, t_y, t_z])
        # pose = (t_x, t_y, t_z)
        # print(pose)
        poses.append(pose)

with open('/media/liaolizhou/Dataset/KITTI/06/results_hist.txt', 'r') as f:
    lines = f.readlines()

query_results = {}

for i in range(len(lines)):
    line = lines[i].strip().split()
    if len(line) == 1:  # 如果行长度为1，即为query
        query = int(line[0])
        query_results[query] = 0
        j = i+1
        # print("j : {0}, length j-th line : {1}".format(j,len(lines[j])))
        while j < len(lines) and len(lines[j].strip().split()) != 1:
            candidate = int(lines[j].strip().split()[1])
            # print(candidate)
            distance = np.linalg.norm(poses[query][0:3] - poses[candidate][0:3])
            if j == i+1:
                print(f"{query}\t{candidate}\t{lines[j].strip().split()[0]}")
            if distance < 5:
                query_results[query] = 1
                break
            j = j + 1
            # print("j : {0}".format(j))

# 将结果写入新文件
with open('/media/liaolizhou/Dataset/KITTI/06/result_hist_true.txt', 'w') as f:
    for query, is_true in query_results.items():
        f.write(f"{query}\t{is_true}\n")
#
# # 输出结果
# for query, is_true in query_results.items():
#     print(f"{query}\t{is_true}")
