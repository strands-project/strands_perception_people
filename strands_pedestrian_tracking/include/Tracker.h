/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _TRACKER_H
#define	_TRACKER_H

#include <map>
#include <string>
#include "Matrix.h"
//#include "Volume.h"
#include "Globals.h"
#include "Hypo.h"
#include "Vector.h"
#include "FrameInlier.h"
#include "Camera.h"
#include "AncillaryMethods.h"
#include "MDL.h"

#include "EKalman.h"
#include "Visualization.h"

#include "CImageHeader.h"

#include <ros/ros.h>

class Tracker
{

    public:
#ifdef cim_v
       void process_tracking_oneFrame(Vector<Hypo>& HyposAll, Detections& allDet, int frame, Vector<Vector<double> >& foundDetInFrame,
                                      CImg<unsigned char>& im, Camera& cam);

       Vector<Hypo> getHyposMDL(){return HyposMDL;}

#else
       void process_tracking_oneFrame(Vector<Hypo>& HyposAll, Detections& allDet, int frame, Vector<Vector<double> > foundDetInFrame,
                                      QImage& im, Camera cam,Matrix<double> depthMap);
#endif
       Tracker();
    private:

    // ------------ Hypos ------------------------------------------------------------------------------------------------------------------------------------------------
    void remove_duplicates(Vector<Hypo>& hypos);
    void prepare_hypos(Vector<Hypo>& vHypos);
    void compute_hypo_entries(Matrix<double>& allX,  Vector<double>& R, Vector<double>& V, Vector<FrameInlier>& Idx, Detections& det, Hypo& hypo, double normfct, int frame);
    void make_new_hypos(int endFrame, int tmin, Detections& det, Vector< Hypo >& hypos,  double normfct, Vector<int>& extendUsedDet);
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //-------------- Trajectories ----------------------------------------------------------------------------------------------------------------------------------------
    void extend_trajectories(Vector< Hypo >& vHypos, Detections& det, int t,int LTPmin,  double normfct , Vector< Hypo >& HypoExtended, Vector<int>& extendUsedDet/*, Camera& cam*/);
    void check_termination(Camera& cam, Vector<Hypo>& Hypos);
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //--------------- Process --------------------------------------------------------------------------------------------------------------------------------------------
    void process_frame(Detections& det, Camera& cam, int t, Vector< Hypo >& HyposAll);
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------

    map<int, int, greater<int> > assignedBBoxCol;
    map<int, int, greater<int> > hypoLastSelForVis;

    Matrix<unsigned char> possibleColors;
    int lastHypoID;

    Vector<Hypo> HyposMDL;
//    Vector<Hypo> HypoEnded;
    Vector<Hypo> hypoStack;
};

#endif	/* _TRACKER_H */

