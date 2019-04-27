import rosbag
import tf
import matplotlib.pyplot as plt
import numpy as np

bag = rosbag.Bag("../dataset/vins_estimation.bag")
# bag.get_type_and_topic_info()

ts = [0]
xyz = [0, 0, 0]
rpy = [0, 0, 0]


is_T0 = False

tf_ros = tf.TransformerROS()  # useful class for transformation


# parsing the bag record from vins estimator (msg type : nav_msgs.msg odometry )
# cf : the camera pose frame of vins estimator is z-frontal coordinate

for topic, msg, t in bag.read_messages(topics=['/vins_estimator/camera_pose']):


    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    q_x = msg.pose.pose.orientation.x
    q_y = msg.pose.pose.orientation.y
    q_z = msg.pose.pose.orientation.z
    q_w = msg.pose.pose.orientation.w

    trans = (x, y, z)
    rot = (q_x, q_y, q_z, q_w)

    T_orig = tf_ros.fromTranslationRotation(trans, rot)

    if not is_T0:
        T0 = T_orig  # referance is the inital transformation
        is_T0 = True
    T = np.linalg.inv(np.matrix(T0)) * np.matrix(T_orig)
    #     T = np.matrix(T_orig)

    # stack time
    ts.append(t.secs + t.nsecs * 1e-9)

    # stack xyz
    xyz = np.vstack((xyz, (T0[0:3, 0:3] * T[0:3, -1]).transpose()))

    # stack rpy
    (roll, pitch, yaw) = tf.transformations.euler_from_matrix(T0[0:3, 0:3] * T[0:3, 0:3] * T0[0:3, 0:3].transpose())
    rpy = np.vstack((rpy, [roll, pitch, yaw]))


# data augmentation for vins (construct Nt x 7 matrix )
ts_vins = np.reshape(np.expand_dims(np.array(ts[1:-1]), -1), (len(ts) - 2, 1))
vins_data_augmented = np.hstack((ts_vins, xyz[1:-1, :], rpy[1:-1, :]))  # [t,x,y,z,roll,pitch,yaw]

np.savetxt('../dataset/vins_estimation.txt',vins_data_augmented,fmt ='%1.4f')

# load to compare with zed and vicon
vicon_data_augmented = np.loadtxt('../dataset/vicon_data.txt')
zed_data_augmented = np.loadtxt('../dataset/zed_estimation.txt')

total_three_data = [vicon_data_augmented, zed_data_augmented, vins_data_augmented]

N_dim = 6  # x,y,z,r,p,y
N_comp = 3

title_set = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
color_set = ['k', (0.5, 0.0, 0.0), (0.0, 0.0, 0.7)]
label_set = ['vicon', 'zed', 'vins']

# plot the three data
for i in range(N_dim):  # iter - dimension
    plt.subplot(6, 1, i + 1)
    plt.title(title_set[i])
    for j in range(N_comp):  # iter - vicon, zed, vins
        plt.plot(total_three_data[j][:, 0] - total_three_data[j][0, 0], total_three_data[j][:, i + 1], c=color_set[j],
                 label=label_set[j])
    plt.legend()
plt.show()
