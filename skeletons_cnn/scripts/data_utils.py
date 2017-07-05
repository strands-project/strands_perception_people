from __future__ import division
#from skimage.io import imsave
import numpy as np
#from skimage import transform
#from skimage.transform import warp
import cv2
#import matplotlib.pyplot as plt

color = [[255, 255, 0], [255, 255, 0],[255,255,0], # color for lower right limb
         [200, 200, 0], [200, 200, 0],[200, 200, 0],# color for lower left limb
         [100, 150, 0], # neck
         [0 ,0, 255], # head
         [0, 255,0],[0, 255,0],[0, 255,0], # color for upper right limb
         [255, 0,255],[255, 0,255],[255, 0,255], # color for upper left limb
         ]

indices = [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15]
bones = [0, 0, 0, 0, 0 ,0 ,0, 0, 0, 1, 1, 0, 1, 1]
part_names = ['r_ankle', 'r_knee','r_hip',
               'l_hip', 'l_knee', 'l_ankle',
               'pelvis','thorax','upper_neck',
               'head_top','r_wrist', 'r_elbow','r_shoulder', 'l_shoulder','l_elbow','l_wrist']

def crop(im, scale, croppos, max_dim):

    shift_to_upper_left = np.identity(3)
    shift_to_center = np.identity(3)
    A = np.identity(3)
    #print(scale)
    scale = float(200/scale) #* float(256/640)
    #print(scale)
    #print(croppos)
    A[0][0] = scale
    A[1][1] = scale
    shift_to_upper_left[0][2] = -croppos[0]
    shift_to_upper_left[1][2] = -croppos[1]
    shift_to_center[0][2] = 128
    shift_to_center[1][2] = 128
    tform = np.matmul(A, shift_to_upper_left)
    tform = np.matmul(shift_to_center, tform)
    w = np.linalg.inv(tform)
    #tform = transform.SimilarityTransform(matrix=tform)
    #im_w = warp(im, tform.inverse, output_shape=(256, 256))
    im_w = cv2.warpAffine(im, tform[0:2,:],(256, 256))
    return im_w, w


def get_scale_max_dim(box):

    scale = box.h
    #print(scale)
    max_dim = np.maximum(box.w, box.h)
    return scale, max_dim


def get_croppos(box):

    X = (box.x + (box.x + box.w))/2
    Y = (box.y + (box.y + box.h))/2
    return [X,Y]


def get_skeleton(preds, tform):
    preds_rewarped = np.ones((3, 16), dtype=np.float32)
    confidence = np.zeros((16), dtype=np.float32)
    for part in range(0, 16):
        coord = np.unravel_index(np.argmax(preds[:, :, part]), [256, 256])
        #print(np.round(coord))
        confidence[part] = np.max(preds[:, :, part])
        preds_rewarped[0, part] = coord[1]
        preds_rewarped[1, part] = coord[0]

    preds_rewarped = np.matmul(tform, preds_rewarped)

    return preds_rewarped, confidence


def draw_skeleton(im, skeleton):

    for part in range(6, 14):

        X = skeleton[0, indices[part]]
        Y = skeleton[1, indices[part]]
        if X > 0 and Y > 0 :
           #print(X)
           #print(Y)
           cv2.circle(im, (X, Y), 8, color[part], -1)
           if bones[part]:
              X2 = skeleton[0, indices[part-1]]
              Y2 = skeleton[1, indices[part-1]]
              if X2 > 0 and Y2> 0 :
                 cv2.line(im, (X, Y),(X2,Y2),(0,0,255),5)
        #print(X)
        #print(Y)


    return im


def draw_skeletons_on_image(im, skeletons, tp):

    for p in range(0, tp):
        draw_skeleton(im, skeletons[p])
    return im


def preprocess(im, persons):

    TotalPersons = len(persons.boxes)
    batch_im = np.zeros((TotalPersons,256,256,3),dtype=np.float32 )
    batch_warp = np.zeros((TotalPersons, 3,3), dtype = np.float32)
    for p in range(TotalPersons):

        #print(persons.boxes[p])
        scale, max_dim = get_scale_max_dim(persons.boxes[p])
        croppos = get_croppos(persons.boxes[p])
        batch_im[p], batch_warp[p] = crop(im, scale, croppos, max_dim)
        path = 'im_' + str(p) + '.jpg'
        #print(croppos)
    return  batch_im, TotalPersons, batch_warp

def get_part_name(idx):
    return  part_names[idx]


