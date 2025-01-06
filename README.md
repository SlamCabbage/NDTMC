# NDT-Map-Code
NDT-Map-Code: A 3D global descriptor for real-time loop closure detection in lidar SLAM

You can find the paper here [NDT-Map-Code](https://arxiv.org/abs/2307.08221).

## News <a name="news"></a>
- `[2024/2]`:fire: NDT-Map-Code is accepted by IROS 2024.

## Demo
[![NDT-Map-Code](https://i.ytimg.com/vi/xCtWRlEKCfk/maxresdefault.jpg)](https://www.youtube.com/watch?v=xCtWRlEKCfk "NDT-Map-Code")

## Example
- We implemented the SLAM package of NDTMC and LIOSAM integration, which can be found at [NDTMC-LIO-SAM](https://github.com/SlamCabbage/NDTMC-LIO-SAM).

- We tested our descriptor on KITTI sequences 00, 02, 05, 06, 07 and 08:

<p align="center"><img width="800" alt="image" src="script/000205.png">

<p align="center"><img width="800" alt="image" src="script/060708.png">

- F1 score and extended precision results on the KITTI dataset:

<p align="center"><img width="800" alt="image" src="script/Table2.png">

## How to use

1. Create GT of LCD

    Use script/create_ground_truth.py to generate the LCD GT value, you need to modify the two file paths in the script/create_ground_truth.py (odometry ground truth file path, LCD GT value result path).

2. Save descriptors

    Modify the main function in run_demo.cpp as follow:

    ```
    int main(int argc, char **argv)
    {
        saveDesc(argv);
        // matchForKitti(argv);
        // matchForNIO(argv);
        return 0;
    }

    ```

    Build and run script/auto_test.py to generate descriptors, where input_folder and output_folder need to be modified to the voledyne folder path of the currently processed sequence and the folder path where you want to save the descriptor results, respectively.

3. Descriptors matching

    Modify the main function in run_demo.cpp as follow:

    ```
    int main(int argc, char **argv)
    {
        // saveDesc(argv);
        matchForKitti(argv);
        // matchForNIO(argv);
        return 0;
    }

    ```

    Build and run script/auto_match.py to perform descriptor matching, where input_folder and output_txt need to be modified to the folder path where the descriptor was saved in the previous step and the txt file path where you want to save the matching result, respectively.

At this point, you can get the matching result of each frame, and you can process it yourself to get more result information, such as: precision-recall curve, F1 Score and Extended Precision.

## Cite NDTMC
```
@inproceedings{liao2024ndt,
  title={NDT-Map-Code: A 3D global descriptor for real-time loop closure detection in lidar SLAM},
  author={Liao, Lizhou and Yan, Wenlei and Sun, Li and Bai, Xinhui and You, Zhenxing and Yuan, Hongyuan and Fu, Chunyun},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={7854--7861},
  year={2024},
  organization={IEEE}
}
```

## Contact
- Maintainer: Lizhou Liao (`liaolizhou@icloud.com`)
#### Contributors
- Lizhou Liao: completed the code

## Acknowledgement
  - Thanks for NIO low-speed localization and mapping group.
