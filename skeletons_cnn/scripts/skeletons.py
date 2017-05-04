#!/usr/bin/env python
# license removed for brevity

import rospy
from mdl_people_tracker.msg import TrackedPersons2d
from sensor_msgs.msg import Image
import message_filters
from approxsync import ApproximateSynchronizer
from cv_bridge import CvBridge
import tensorflow as tf
from network import inception
from data_utils import preprocess,get_skeleton, draw_skeletons_on_image,get_part_name
import numpy as np
import skimage.io as io
import time
from skeletons_cnn.msg import skeleton, joint
import geometry_msgs.msg

bridge = CvBridge()
sess = tf.Session()
X = tf.placeholder(tf.float32, shape=None)
logits = inception(X,False)
preds = tf.nn.sigmoid(logits)
saver = tf.train.Saver()

teaser_batch = np.zeros([8, 256, 256,3], dtype = np.float32)

skel_im_pub = rospy.Publisher('/skeletons_cnn/image_with_skeletons', Image, queue_size=1)
skeleton_pub = rospy.Publisher('/skeletons_cnn/skeleton', skeleton, queue_size=1)


def callback(Im , Persons):

    cv2_img = bridge.imgmsg_to_cv2(Im, "rgb8")
    batch_im, tp, batch_warp = preprocess(cv2_img, Persons)
    skeletons = np.ones((tp, 3, 16), dtype=np.float32)
    confidence = np.zeros((tp, 16), dtype=np.float32)
    for p in range(0, tp):
           teaser_batch[p] = np.divide(batch_im[p],255.0)
    output = sess.run(preds, feed_dict={X:teaser_batch})

    for p in range(0, tp):
        s = skeleton()
        skeletons[p], confidence[p] = get_skeleton(output[p], batch_warp[p])
        s.userID = Persons.boxes[p].track_id
        for j in range(0, 16):
            part = joint()
            pose = geometry_msgs.msg.Pose()
            pose.position.x = skeletons[p, 0, j]
            pose.position.y = skeletons[p, 1, j]
            part.pose = pose
            part.confidence = confidence[p, j]
            part.name = get_part_name(j)
            s.joints.append(part)
        s.time = rospy.get_rostime()
        skeleton_pub.publish(s)

    if tp > 0 :

        cv2_img = draw_skeletons_on_image(cv2_img, skeletons, tp)
        print('Processesed batch')
        pub_im = bridge.cv2_to_imgmsg(cv2_img,"rgb8")
        skel_im_pub.publish(pub_im)
    else :
       print('No detections')
       skel_im_pub.publish(Im)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    ns = rospy.get_name() + '/'
    network = rospy.get_param(ns + 'network_path')
    network_path = network + 'pose_net.chkp'
    rgb_image = '/head_xtion/rgb/image_rect_color'
    tracked_persons = '/mdl_people_tracker/tracked_persons_2d'
    saver.restore(sess, network_path)

    teaser_images_folder = rospy.get_param(ns + 'teaser_batch_path')

    print('Initializing and running on teaser batch')
    for t in range(1,7):
        path = teaser_images_folder + 'teaser' + str(t) + '.jpg'
        teaser_batch[t] = np.divide(np.array(io.imread(path)),255.0)
    output = sess.run(preds, feed_dict={X: teaser_batch})
    print('initialization done')
    print('Ready')

    Image_sub = message_filters.Subscriber(rgb_image, Image)
    TrackedPersons2d_sub = message_filters.Subscriber(tracked_persons, TrackedPersons2d)


    syncSlop = rospy.get_param("~sync_slop",
                               0.2)  # in seconds; messages closer in time than this are output by the approximate time synchronizer immediately
    syncQueueSize = rospy.get_param("~sync_queue_size", 1)  # size of the synchronizer queue
    timeSynchronizer = ApproximateSynchronizer(syncSlop, [Image_sub, TrackedPersons2d_sub], syncQueueSize)


    timeSynchronizer.registerCallback(callback)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
