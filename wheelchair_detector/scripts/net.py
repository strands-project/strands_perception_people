#!/usr/bin/env python

import numpy as np
import DeepFried2 as df


def slot(nin, nout, fs, bn=True):
    kw = {'bias': False if bn else df.init.const(0)}
    conv = df.SpatialConvolutionCUDNN(nin, nout, (1,fs), border='valid', init=df.init.ortho_svd(), **kw)
    bn = df.BatchNormalization(nout)
    nl = df.ReLU()
    do = df.Dropout(0.25)
    return (conv, bn, nl, do) if bn else (conv, nl, do)

def mknet(bn, win_res):
    net = df.Sequential()

    # Add singleton "channel" and singleton "height" dimensions.
    net.add(df.Reshape(-1, 1, 1, win_res))
    net.add(*slot(  1, 64, 5, bn))
    net.add(*slot( 64, 64, 5, bn))
    net.add(df.PoolingCUDNN((1,2)))

    net.add(*slot( 64, 128, 5, bn))
    net.add(*slot(128, 128, 3, bn))
    net.add(df.PoolingCUDNN((1,2)))

    net.add(*slot(128, 256, 5, bn))
    # Is now (X, 256, 1, 3)

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

def load_net_from_file(fname,bn=True, win_res=48):
    net = mknet(bn=bn, win_res=win_res)
    net.__setstate__(np.load(fname)['arr_0'])
    net.evaluate()
    return net
