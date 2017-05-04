import tensorflow as tf
import tensorflow.contrib.slim as slim
import skimage.io as io
import numpy as np
import matplotlib.pyplot as plt

def inception_module(x, config, reuse, name):

    image = tf.placeholder(tf.float32, shape=None)
    with slim.arg_scope([slim.conv2d], padding='SAME', activation_fn=tf.nn.elu,
                        normalizer_fn=slim.batch_norm, stride=1):
        if config[0][0] != 0 :
           branch1 = slim.layers.conv2d(x,
                                      config[0][0], [1, 1],scope= name + '/' + 'branch1',reuse=reuse)#ConvBN(input, [1, 1], [config[0][0], 1])

        branch3 = slim.layers.conv2d(x,config[1][0], [1, 1],scope=name + '/' +'branch3_1',reuse=reuse)#ConvBN(input, [1, 1], [config[1][0], 1])
        branch3 = slim.layers.conv2d(branch3,config[1][1], [3, 3],scope=name + '/' +'branch3_2',reuse=reuse)#ConvBN(branch3, [1, 1], [config[1][1], 3])

        branch3x3 = slim.layers.conv2d(x,config[2][0], [1, 1],scope=name + '/' +'branch3x3_1',reuse=reuse)#ConvBN(input, [1, 1], [config[2][0], 1])
        branch3x3 = slim.layers.conv2d(branch3x3,config[2][1], [3, 3],scope=name + '/' +'branch3x3_2',reuse=reuse)#ConvBN(branch3x3, [1, 1], [config[2][1], 1])
        branch3x3 = slim.layers.conv2d(branch3x3,config[2][1], [3, 3],scope=name + '/' +'branch3x3_3',reuse=reuse)#ConvBN(branch3x3, [1,1], [config[2][1], 1])

        if config[3][0] == 0 :
            branch_pool = slim.layers.max_pool2d(x,3,1,padding='SAME', scope=name + '/' +'branch_pool')#tf.nn.max_pool(input, ksize=[1, 3, 3, 1], strides=[1, 1, 1, 1], padding='SAME', name = 'pool')
        else:
            branch_pool = slim.layers.avg_pool2d(x,3,1,padding='SAME', scope=name + '/' +'branch_pool')#tf.nn.avg_pool(input, ksize=[1, 3, 3, 1], strides=[1, 1, 1, 1], padding='SAME', name='pool')
        if config[3][1] != 0:
            branch_pool =slim.layers.conv2d(branch_pool,config[3][1],[1,1],scope=name + '/' +'branch_pool_conv',reuse=reuse) #ConvBN(branch_pool, [1, 1], [config[3][1],1])

        if config[0][0] != 0 :
           mixed = tf.concat(axis=3, values=[branch1, branch3, branch3x3, branch_pool])
        else :
           mixed = tf.concat(axis=3, values=[branch3, branch3x3, branch_pool])
    return mixed

def inception(batch, train=True):

    if train:
        reuse = None
    else:
        reuse = True

    ops = {}
    image = tf.placeholder(tf.float32, shape=None)
    # Pre-processing the input
    with tf.name_scope('preprocessing'):
        ops['preprocessing'] = tf.div(batch, 255)
    with tf.name_scope('reshape_1'):
        ops['reshape_1'] = tf.reshape(ops['preprocessing'], [-1, 256, 256, 3])
    # Pre-inception
    with slim.arg_scope([slim.conv2d], padding='SAME', activation_fn=tf.nn.elu,
                          normalizer_fn=slim.batch_norm, stride=1):
        ops['conv1'] = slim.layers.conv2d(ops['reshape_1'],
                                      64, [7, 7], stride=2,scope='conv1',
                                      reuse=reuse)
        ops['pool1'] = slim.layers.max_pool2d(ops['conv1'],3, 2, padding='SAME',scope='pool1')
        ops['conv2'] = slim.layers.conv2d(ops['pool1'],
                                      64, [1, 1], scope='conv2',
                                      reuse=reuse)
        ops['conv3'] = slim.layers.conv2d(ops['conv2'],
                                          192, [1, 1], scope='conv3',
                                          reuse=reuse)
        ops['pool2'] = slim.layers.max_pool2d(ops['conv3'],3, 2, padding='SAME', scope='pool2')


    # First block of inception layers
    ops['incept3a'] = inception_module(ops['pool2'], [[64], [64, 64], [64, 96], [1, 32]], reuse, 'incept3a')
    ops['incept3b'] = inception_module(ops['incept3a'], [[64], [64, 64], [64, 96], [1, 64]],reuse,'incept3b')
    ops['incept3c'] = inception_module(ops['incept3b'], [[0], [128, 60], [64, 96], [0, 0]],reuse, 'incept3c')
    ops['pool3'] = slim.layers.max_pool2d(ops['incept3c'],3, 2, padding='SAME', scope='pool3')#tf.nn.max_pool(ops['incept3c'], ksize=[1, 3, 3, 1], strides=[1, 2, 2, 1], padding='SAME',
    ops['incept4a'] = inception_module(ops['pool3'], [[224], [64, 96], [96, 128], [1, 128]],reuse, 'incept4a')
    ops['incept4b'] = inception_module(ops['incept4a'], [[192], [96, 128], [96, 128], [1, 128]],reuse,'incept4b')
    ops['incept4c'] = inception_module(ops['incept4b'], [[160], [128, 160], [128, 160], [1, 96]],reuse, 'incept4c')
    ops['incept4d'] = inception_module(ops['incept4c'], [[96], [128, 192], [160, 192], [1, 96]],reuse, 'incept4d')

    # 2nd block of inception layers
    ops['incept4e'] = inception_module(ops['incept4d'], [[0], [128, 192], [192, 256], [0, 0]], reuse, 'incept4e')
    ops['pool4'] =slim.layers.max_pool2d(ops['incept4e'],3, 2, padding='SAME', scope='pool4')
    ops['incept5a'] = inception_module(ops['pool4'], [[352], [192, 320], [160, 224], [1, 128]], reuse,'incept5a')
    ops['incept5b'] = inception_module(ops['incept5a'], [[352], [192, 320], [192, 224], [0, 128]], reuse,'incept5b')

    # Upsample the output of 2nd block to the resolution of first block
    upsample1 = slim.layers.conv2d_transpose(ops['incept5b'], 576, [2,2], 2 ,
                                                  activation_fn=tf.nn.elu,
                                                  normalizer_fn=slim.batch_norm, scope='dconv1', reuse=reuse)#deConvBN(ops2['incept_5b'], [2, 2], [576, 2])
    ops['concat'] = tf.concat(axis=3, values=[tf.identity(ops['incept4d']), upsample1])
    ops['dconv'] = slim.layers.conv2d_transpose(ops['concat'], 16, [32, 32], 16 ,
                                                  activation_fn=None,
                                                  normalizer_fn=None, scope='dconv2', reuse=reuse)

    #saver = tf.train.Saver()
    #saver.restore(sess, '../pose_model/pose_net.chkp')

    #ops['preds'] = tf.nn.sigmoid(ops['dconv'])

    #output =sess.run(ops['preds'], feed_dict={image:images_batch})


    return ops['dconv']

def main():

    im = np.array(io.imread('im_2.jpg'))
    im = im[...,::-1]
    im = np.reshape(im, [1, 256, 256, 3])
    im = np.divide(im, 255.0)
    X = tf.placeholder(tf.float32, shape=None)
    logits = inception(X,False)
    preds = tf.nn.sigmoid(logits)

    sess = tf.Session()

    saver = tf.train.Saver()
    saver.restore(sess, '../pose_model/pose_net.chkp')

    output = sess.run(preds, feed_dict={X:im})

    plt.imshow(im[0])

    for i in range(7,14):

        coord = np.unravel_index(np.argmax(output[0,:,:,i]),[256, 256])
        plt.scatter(coord[1], coord[0])

    plt.show()


if __name__ == '__main__':
    main()
