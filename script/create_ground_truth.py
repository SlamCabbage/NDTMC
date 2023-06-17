import numpy
import numpy as np
import math

poses = []
with open('/home/liaolizhou/dataset/poses/07.txt', 'r') as f:
    for line in f:
        numbers = line.strip().split()
        t_x, t_y, t_z = float(numbers[3]), float(numbers[7]), float(numbers[11])
        pose = np.array([t_x, t_y, t_z])
        # pose = (t_x, t_y, t_z)
        print(pose)
        poses.append(pose)

f = open('/media/liaolizhou/Dataset/KITTI/07/poses_gt.txt','w')

for i in range(50, len(poses)):
    is_true = False
    for j in range(0, i-49):
        # distance = np.linalg.norm(poses[i][0:3] - poses[j][0:3])
        distance = math.sqrt((poses[i][0]-poses[j][0])*(poses[i][0]-poses[j][0]) + (poses[i][1]-poses[j][1])*(poses[i][1]-poses[j][1]) + (poses[i][2]-poses[j][2])*(poses[i][2]-poses[j][2]))
        if distance < 5:
            is_true = True
            break

    if is_true:
        f.write(f"{i}\t1\n")
    else:
        f.write(f"{i}\t0\n")
