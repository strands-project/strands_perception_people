## Skeletons CNN

This package predicts 2D skeletons of multiple persons and requires the persons to be tracked before hand by the bayes ppl tracker.

Mandatory :

 Download and extract the model from https://drive.google.com/open?id=0B_UCKnAjgo-cV0pHbktvY3ZMN0U in the pose_model folder.

Dependencies :

It has the follwoing dependencies.

1) A mid range gpu e.g GTX 1050 Ti
2) Download and install cuda 8.0 for Linux Ubuntu 14.04 64-bit from this page: https://developer.nvidia.com/cuda-downloads
3) Install CuDNN

    Go to https://developer.nvidia.com/cuDNN and use the Download button (you have to register and login to download. no way around that.)
    Download cuDNN 5.1 for Linux. You will download a file cudnn-8.0-linux-x64-v5.1.tgz then use the commands:
    
    i ) tar -xvf cudnn-7.0-linux-x64-v3.0-prod.tgz
    ii) sudo cp cuda/include/*.h /usr/local/cuda/include
    iii) sudo cp cuda/lib64/*.so* /usr/local/cuda/lib64

4)  Install Tensor flow 1.0.1 with Virtual Environment as follows:

 i) Install pip and virtualenv by issuing the following command:

         $ sudo apt-get install python-pip python-dev python-virtualenv 

 ii) Create a virtualenv environment by issuing the following command:

         $ virtualenv --system-site-packages targetDirectory 

     The targetDirectory specifies the top of the virtualenv tree. Our instructions assume that targetDirectory is ~/tensorflow, but you may choose any directory.

 iii) Activate the virtualenv environment by issuing one of the following commands:

         $ source ~/tensorflow/bin/activate # bash, sh, ksh, or zsh
         $ source ~/tensorflow/bin/activate.csh  # csh or tcsh

     The preceding source command should change your prompt to the following:

         (tensorflow)$  

  iv)  install TensorFlow in the active virtualenv environment by issuing a command of the following format:
   
         pip install --upgrade https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-1.0.1-cp27-none-linux_x86_64.whl 

   v)  Validate the tensor flow installation as follows
        
       Open a new terminal and activate the virtual environment for tensor flow as 
         $ source ~/tensorflow/bin/activate # bash, sh, ksh, or zsh
       
       Invoke python from your shell as follows:
         $ python
       
       Enter the following short program inside the python interactive shell:

         >>> import tensorflow as tf
	 >>> hello = tf.constant('Hello, TensorFlow!')
	 >>> sess = tf.Session()
	 >>> print(sess.run(hello))

Required Topics :

It needs the following topics as inputs

RGB Image Topic : /head_xtion/rgb/image_rect_color <br />
Tracked Persons Topic: /mdl_people_tracker/tracked_persons_2d <br />
TODO : Depth Image Topic: /head_xtion/depth/image_rect <br />
  


Running the package:
 
   i)  Launch the bayes people tracker with mdl tracker as follows <br />
       perception_people_launch people_tracker_robot.launch with_mdl_tracker:=true 

   ii) Open a new terminal and activate the virtual environment for tensor flow <br />
         source ~/tensorflow/bin/activate # bash, sh, ksh, or zsh

   iii) Launch the skeletons package inside the virtual environment as follows <br />
        roslaunch skeletons_cnn pose_cnn.launch 



skeletons.msg from package skeletons_cnn (in python 'from skeletons_cnn.msg import skeleton'):
 
    It outputs the following skeleton.msg with the following format
    Header header
    int32 userID
    joint[] joints
    time time

TODO : The current version publishes 2D skeletons. Will add the depth image topic and convert 2D skeleton to 3D tommorow.




