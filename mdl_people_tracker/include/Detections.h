/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _DENNIS_DETECTIONS_H
#define	_DENNIS_DETECTIONS_H

#include "Camera.h"
#include "Math.h"

#include "CImageHeader.h"

#include <ros/ros.h>

#ifndef cim_v
#include <QImage>
#include <QColor>
#endif

//***************************************************************************************
// Detections are stored in detC as follows
//
//         frame                   detections
//          | |                    | |
//          | |                    | |
//          | |                    | |                      infos about det i.e. 3D pos
// Vector   | |----------> Vector  | |-------------------->| | | | | | | | | | |
//          | |                    | |
//          | |
//***************************************************************************************


//+++++++++++++++++++++++++++++++ Definition +++++++++++++++++++++++++++++++++
class Detections
{
public:


//    Detections(Matrix <double>& detO, const int flag, double dThresh, bool HOG);
    Detections(int x, const int flag);
//    Detections(Detections& det1, Detections& det2, int flag);

    ~Detections();

    //*******************************************************
    // getter methods
    //*******************************************************
//    int numberFrames();
    int numberDetectionsAtFrame( int frame);
    void getPos3D( int frame,  int detec, Vector<double>& pos);
    void getBBox( int frame,  int detec, Vector<double>& bbox);
    double getScore( int frame,  int detec);
    double getHeight( int frame,  int detec);
//    void getDetection(int frame, int detec, Vector<double>& det);
    int getCategory(int frame, int detec);
    uint32_t getSeqNr(int frame, int detec);

    int getIndex(int frame, int detec);

//    int getDetNumber(int frame, int detec);
//    Vector<Vector<double> > get3Dpoints(int frame, int detec);
//    Vector<Vector<int> > getOccCells(int frame, int detec);

    //*******************************************************
    // setter methods
    //*******************************************************

//    void setScore(int frame, int pos, double scoreValue);

    //*************************************************************
    // Methode for adding online Detections
    //*************************************************************

#ifdef cim_v
    void addHOGdetOneFrame(Vector<Vector <double> >& det, int frame, CImg<unsigned char>& imageLeft, Camera cam);
#else
    void addHOGdetOneFrame(Vector<Vector <double> >& det, int frame, QImage& imageLeft, Camera cam, Matrix<double>& depth);
#endif

    int prepareDet(Vector<double> &detContent, Vector<Vector <double> >& det, int i, bool leftDet,
                   Camera cam, Matrix<double> &covariance);

    //*****************************************************************
    // Compute 3D Position out of BBox
    //*****************************************************************
    void compute3DPosition(Vector<double>& detection, Camera cam);
    //*****************************************************************
    // Compute 3D Position out of BBox
    //*****************************************************************
#ifdef cim_v
    void computeColorHist(Volume<double>& colHist, Vector<double>& bbox, int nBins, CImg<unsigned char>& imageLeft);
#else
    void computeColorHist(Volume<double>& colHist, Vector<double>& bbox, int nBins, QImage& imageLeft);
#endif
    void getColorHist(int frame, int pos, Volume<double>& colHist);
    //*****************************************************************
    // Compute the 3D uncertainty for a point
    //*****************************************************************
    void compute3DCov(Vector<double> pos3d, Matrix<double> &cov);
    void get3Dcovmatrix(int frame, int pos, Matrix<double>& covariance);

    Vector<double> fromCamera2World(Vector<double> posInCamera, Camera cam);
    bool improvingBBoxAlignment_libelas(Vector<double>& vbbox, double var, Camera camera, Matrix<double>& depthMap);
    Vector<double> projectPlaneToCam(Vector<double> p, Camera cam);

    double get_mediandepth_inradius(Vector<double>& bbox, int radius, Matrix<double>& depthMap, double var, double pOnGp);

protected:

    Vector< Vector < Vector  <double> > > detC;
    Vector< Vector < Matrix  <double> > > cov3d;
    Vector<Vector<Volume<double> > > colHists;
//    Vector<Vector<double> > gp;
//    Vector<Vector<Vector<Vector<double> > > > points3D_;
//    Vector<Vector<Vector<Vector<int> > > > occBins_;

    Camera camLCov;
    Camera camRCov;

//    int sizeOfDetVec;
    int offSet;
    int img_num, hypo_num, center_x, center_y, scale, categ, bbox, initscore,
    score, dist, height, rot, pos, numberAllAccDetections, numberOfFrames, nrColinDetFile, carOrient, det_id;

};

#endif	/* _DENNIS_DETECTIONS_H */

