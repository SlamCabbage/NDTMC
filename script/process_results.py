import numpy as np

poses = []
with open('/home/liaolizhou/dataset/poses/00.txt', 'r') as f:
    for line in f:
        numbers = line.strip().split()
        t_x, t_y, t_z = float(numbers[3]), float(numbers[7]), float(numbers[11])
        pose = np.array([t_x, t_y, t_z])
        # pose = (t_x, t_y, t_z)
        print(pose)
        poses.append(pose)

with open('/media/liaolizhou/Dataset/KITTI/00/results_1m_average.txt', 'r') as f:
    lines = f.readlines()

query_results = {}

for i in range(len(lines)):
    line = lines[i].strip().split()
    if len(line) == 1:  # 如果行长度为1，即为query
        query = int(line[0])
        # if(i+1 < len(lines) and len(lines[i+1].strip().split()) != 1):
        top_result = lines[i+1].strip().split()[0:2]  # 提取top1检索
        query_results[query] = top_result

# 将结果写入新文件
with open('/media/liaolizhou/Dataset/KITTI/00/results_top1_1m_average.txt', 'w') as f:
    for query, top_result in query_results.items():
        candidate = int(top_result[0])
        distance = np.linalg.norm(poses[query][0:3] - poses[candidate][0:3])
        f.write(f"{query}\t{top_result[0]}\t{top_result[1]}\t{distance}\n")

# 输出结果
for query, top_result in query_results.items():
    print(f"{query}\t{top_result}")
