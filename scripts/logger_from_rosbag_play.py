#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import tf.transformations
import rospy


if __name__=="__main__":

    # we compare the VO result from vins mono and zed with vicon data
    rospy.init_node('logger_from_rosbag_play')
    rospy.loginfo('logger initiated.')

    is_Tmz_t_received = False # original information from zed (map to zed)
    is_Twv_t_received = False # original information from vicon (world to vicon_drone)

    ts_vicon = [0] ; tranl_vicon = [0,0,0] ; rot_vicon = [0,0,0] # 3 x Nt for transl and 3 x Nt for rot
    ts_zed = [0]; tranl_zed = [0,0,0]  ; rot_zed = [0,0,0]

    vicon_tf_id = '/vicon/felipe_drone/felipe_drone'
    zed_vo_tf_id = 'zed_camera_center'
    listener = tf.TransformListener() # listener for get frame
    tf_ros = tf.TransformerROS() # useful class
    Tmzs = []
    Twvs = []
    rate = rospy.Rate(50.0)

    while not rospy.is_shutdown():


        # world to map transform
        try:
            (trans, rot) = listener.lookupTransform('/world', '/map', rospy.Time(0))
            Twm0 = tf_ros.fromTranslationRotation(trans, rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        # zed retrieve
        try:
            (trans, rot) = listener.lookupTransform('/map',zed_vo_tf_id, rospy.Time(0))
            Tmz = tf_ros.fromTranslationRotation(trans, rot)
            Tmzs.append(Tmz)
            if not is_Tmz_t_received:
                print 'zed estimation received'
            is_Tmz_t_received = True

            t = listener.getLatestCommonTime(zed_vo_tf_id, '/map')
            ts_zed.append(t.secs+t.nsecs*1e-9)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # vicon retrieve
        try:
            (trans, rot) = listener.lookupTransform('/world',vicon_tf_id, rospy.Time(0))
            # print trans
            Twv = tf_ros.fromTranslationRotation(trans, rot)

            Twvs.append(Twv)
            if not is_Twv_t_received:
                print 'vicon estimation received'
            is_Twv_t_received = True

            t = listener.getLatestCommonTime(vicon_tf_id, '/world')
            ts_vicon.append(t.secs+t.nsecs*1e-9)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()


    # transformation from the initial coordinate for the both

    # zed
    for i in range(len(Tmzs)):
        Tmz0_inv = np.matrix(np.zeros((4,4)))
        Tmz0_inv[3,3] = 1
        Tmz0_inv[0:3,0:3] = Tmzs[0][0:3,0:3].transpose()
        Tmz0_inv[0:3,-1] = np.matrix(-Tmzs[0][0:3,0:3].transpose())*np.matrix(Tmzs[0][0:3,-1]).transpose()

        T = Tmz0_inv * np.matrix(Tmzs[i]) # caution: matrix type conversion is required


        # get xyz
        tranl_zed = np.vstack((tranl_zed, (np.matrix(Twm0[0:3,0:3])*np.matrix(Tmzs[0][0:3,0:3])*T[0:3, -1]).transpose()))

        # get rpy
        (r, p, y) = tf.transformations.euler_from_matrix((Tmzs[0][0:3,0:3]*T[0:3, 0:3]*Tmz0_inv[0:3,0:3]))
        rot_zed = np.vstack((rot_zed, [r, p, y]))


        if i == 0:
            print tranl_zed[1,:]


    # vicon
    for i in range(len(Twvs)):
        T = np.linalg.inv(np.matrix(Twvs[0])) * np.matrix(Twvs[i])
        # T = np.matrix(Twvs[i])

        # get xyz
        tranl_vicon = np.vstack((tranl_vicon, ((Twvs[0][0:3,0:3])*T[0:3, -1]).transpose()))
        # tranl_vicon = np.vstack((tranl_vicon, (T[0:3, -1]).transpose()))

        # get rpy
        (r, p, y) = tf.transformations.euler_from_matrix(T[0:3, 0:3])
        rot_vicon = np.vstack((rot_vicon, [r, p, y]))

    # logging to txt file

    # zed
    # print np.shape(np.expand_dims(np.array(ts_zed[1:-1]),-1))
    # print len(ts_zed)
    ts_zed_col = np.reshape(np.expand_dims(np.array(ts_zed[1:-1]),-1),(len(ts_zed)-2,1))
    zed_data_augmented = np.hstack((ts_zed_col,tranl_zed[1:-1,:],rot_zed[1:-1,:]))
    np.savetxt('zed_estimation.txt',zed_data_augmented,fmt ='%1.4f')
    print 'zed_estimation.txt saved'

    # vicon
    ts_vicon_col = np.reshape(np.expand_dims(np.array(ts_vicon[1:-1]),-1),(len(ts_vicon)-2,1))
    vicon_data_augmented = np.hstack((ts_vicon_col,tranl_vicon[1:-1,:],rot_vicon[1:-1,:]))
    np.savetxt('vicon_data.txt',vicon_data_augmented,fmt ='%1.4f')
    print 'vicon_data.txt saved'



    # plot the result
    plt.subplot(611)
    plt.title('x')
    plt.plot(ts_vicon[1:-1],tranl_vicon[1:-1,0],'r',label = 'vicon')
    plt.plot(ts_zed[1:-1],tranl_zed[1:-1,0],'b',label = 'zed')
    plt.legend()


    plt.subplot(612)
    plt.title('y')
    plt.plot(ts_vicon[1:-1],tranl_vicon[1:-1,1],'r',label = 'vicon')
    plt.plot(ts_zed[1:-1],tranl_zed[1:-1,1],'b',label = 'zed')
    plt.legend()


    plt.subplot(613)
    plt.title('z')
    plt.plot(ts_vicon[1:-1],tranl_vicon[1:-1,2],'r',label = 'vicon')
    plt.plot(ts_zed[1:-1],tranl_zed[1:-1,2],'b',label = 'zed')
    plt.legend()


    plt.subplot(614)
    plt.title('roll')
    plt.plot(ts_vicon[1:-1],rot_vicon[1:-1,0],'r',label = 'vicon')
    plt.plot(ts_zed[1:-1],rot_zed[1:-1,0],'b',label = 'zed')
    plt.legend()

    plt.subplot(615)
    plt.title('pitch')
    plt.plot(ts_vicon[1:-1],rot_vicon[1:-1,1],'r',label = 'vicon')
    plt.plot(ts_zed[1:-1],rot_zed[1:-1,1],'b',label = 'zed')
    plt.legend()

    plt.subplot(616)
    plt.title('yaw')
    plt.plot(ts_vicon[1:-1],rot_vicon[1:-1,2],'r',label = 'vicon')
    plt.plot(ts_zed[1:-1],rot_zed[1:-1,2],'b',label = 'zed')
    plt.legend()

    plt.show()


