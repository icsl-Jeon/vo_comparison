# vo_comparison

____

<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/rviz.png">

<img src="https://github.com/icsl-Jeon/traj_gen/blob/master/img/vicon_zed_vins.png">

This package compare different VO algorithms(VO from ZED stereo labs and vins fusion from HKUST) with VICON sensor data.

Included features are 

1. Integrated zed mini camera and vicon launch file with static tf publisher (/map frame and /world frame is connected)
2. config file for zed to be used in vins-fusion is written   



## 1. Dependencies 

---



### ZED ros wrapper

[download here](https://github.com/stereolabs/zed-ros-wrapper)  

### vicon_bridge

[download here](https://github.com/ethz-asl/vicon_bridge)  

### vins-fusion (extended version from vins-mono for stereo IMU)

[download here](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)  



## 2. Usage 

____



### 2.1 build your own dataset  (refer ./dataset folder )

Record with rosbag the followings: **/odom** (zed estimation) , **/clock** (for time sync!), **/imu/data**, **/left/image_raw_color**, **/right/image_raw_color** , **/vicon_pose**. Do not record **/tf** directly!! 

```
roslaunch zed_vicon_comp.launch 
```

A dataset made by me can be found [here](https://drive.google.com/open?id=1ztOUUuxVBkayTsWErh2n-K4R_Pmuosu-)

### 2.2 Run VO algorithm (vins-fusion here) from rosbag play 

​	I prepared the config file for vins-fusion. Also, rosbag record for the output of VO algorithm 

```
rosrun vins vins_node path/to/mav_vo_comp/vins_fusion_config/zed_imu_conf/yaml
```



### 2.3 Compare the algorithms with vicon and zed odometry

​	Let's analyze  the rosbag file . Go to /scripts folder 

 * ```
   rosrun mav_vo_comp logger_rosbag_play.py
   ```

   Run the script and then rosbag play the *.bag file, which will leave *zed_estimation.txt* and *vicon_data.txt* 

   The .*txt file contains a matrix of shape Nt x 7 (column: (t ,x,y,z,roll,pitch,yaw)).   

 * ``` 
   python vicon_zed_vins_comp.py
   ```

   This intakes *zed_estimation.txt* and *vicon_data.txt*  and *vins_estimator.bag*. Then compare the three and plot.

    

## 3. Caution 

___



+ The zed mini config file for the vins fusion is valid only for the zed camera attached to my drone(SN10026992.conf). If you want use your camera, Then a your own calibration file is referred which can be found /usr/loca/zed/settings/*.conf
+ The image should be recorded with VGA format(672x376)
+   Do not record /tf directly! it really bothers to parse the data in offline script. In the tf recorded in rosbag file, the transformation information is included in random order. Also, arbitrary query from A_frame to B_frame is difficult... If one recorded /tf, then he should parse the data with tf listener receiving tf from rosbag play (it's me actually.)  