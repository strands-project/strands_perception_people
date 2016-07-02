#!/usr/bin/env python

import numpy as np
import DeepFried2 as df


def slot(nin, nout, fs, cudnn, bn=True):
    kw = {'bias': False if bn else df.init.const(0)}
    if cudnn:
        conv = df.SpatialConvolutionCUDNN(nin, nout, (1,fs), border='valid', init=df.init.ortho_svd(), **kw)
    else:
        conv = df.SpatialConvolution(nin, nout, (1,fs), border='valid', init=df.init.ortho_svd(), **kw)
    bn = df.BatchNormalization(nout)
    nl = df.ReLU()
    do = df.Dropout(0.25)
    return (conv, bn, nl, do) if bn else (conv, nl, do)

def mknet(bn, win_res, cudnn):
    net = df.Sequential()

    # Add singleton "channel" and singleton "height" dimensions.
    net.add(df.Reshape(-1, 1, 1, win_res))
    net.add(*slot(  1, 64, 5, cudnn, bn))
    net.add(*slot( 64, 64, 5, cudnn, bn))
    net.add(df.PoolingCUDNN((1,2)) if cudnn else df.SpatialMaxPooling((1,2)))

    net.add(*slot( 64, 128, 5, cudnn, bn))
    net.add(*slot(128, 128, 3, cudnn, bn))
    net.add(df.PoolingCUDNN((1,2)) if cudnn else df.SpatialMaxPooling((1,2)))

    net.add(*slot(128, 256, 5, cudnn, bn))
    # Is now (X, 256, 1, 3)

    if cudnn:
        net.add(
            df.Parallel(
                # Confidence
                df.Sequential(
                    df.SpatialConvolutionCUDNN(256, 3, (1,3), init=df.init.const(0)),
                    df.Reshape(-1, 3),
                    df.SoftMax(),
                ),
                # Offset "vote"
                df.Sequential(
                    df.SpatialConvolutionCUDNN(256, 2, (1,3), init=df.init.const(0)),
                    df.Reshape(-1, 2),
                ),
            )
        )
    else:
        net.add(
            df.Parallel(
                # Confidence
                df.Sequential(
                    df.SpatialConvolution(256, 3, (1,3), init=df.init.const(0)),
                    df.Reshape(-1, 3),
                    df.SoftMax(),
                ),
                # Offset "vote"
                df.Sequential(
                    df.SpatialConvolution(256, 2, (1,3), init=df.init.const(0)),
                    df.Reshape(-1, 2),
                ),
            )
        )
    #print("{:.3f}M params".format(df.utils.count_params(net)/1000.0/1000.0))
    return net

def load_net_from_file(fname, cudnn,bn=True, win_res=48):
    net = mknet(bn=bn, win_res=win_res, cudnn=cudnn)
    net.__setstate__(np.load(fname)['arr_0'])
    net.evaluate()
    return net
