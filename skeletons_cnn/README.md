## Skeletons CNN

This package predicts 2D the upper body skeletons of multiple persons and requires the persons to be tracked before hand by the mdl tracker.

Dependencies :

The current version has the follwoing dependencies.

1) GTX 1050 GPU
2) Cuda 8.0
3) cudnn 5.1
4) Tensor flow 1.0.1. Install Tensor flow with Virtual Environment as follows:

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

RGB Image Topic :
Tracked Persons Topic:
Depth Image Topic:
  


Running the package:

   Open a new terminal and activate the virtual environment for tensor flow 
         $ source ~/tensorflow/bin/activate # bash, sh, ksh, or zsh
   Launch the Mdl Person tracker
   Launch the Skeleton estimator


Skeletons:Msg :
 
    It outputs the following msg 






