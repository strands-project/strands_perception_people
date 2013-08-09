/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _DENNIS_KALMAN_H
#define	_DENNIS_KALMAN_H

#include <string.h>

#include "Globals.h"
#include "Vector.h"
#include "Matrix.h"
#include "Volume.h"
#include "FrameInlier.h"
//#include <QImage>
//#include <QColor>
//#include <QPainter>
#include "Kalman.h"
#include "Detections.h"


class EKalman : public Kalman
{
public:

    EKalman();
    void runKalmanUp(Detections& det, int frame, int t, Matrix<double>& allXnew, Vector<FrameInlier>& Idx,
                                Vector<double>& R, Vector<double>& Vel, Volume<double>& hMean,  Vector<Matrix<double> >& stateCovMats, Volume<double>& colHistInit,
                                Vector<Volume<double> >& colHists, double yPosOfStartingPoint, Vector<double> &bbox);

    void runKalmanDown(Detections& det, int frame, int pointPos, int t, Matrix<double>& allXnew, Vector<FrameInlier>& Idx,
                       Vector<double>& R, Vector<double>& Vel, Volume<double>& hMean, Vector<Matrix<double> >& stateCovMats,
                       Vector<Volume<double> >& colHists);

    bool findObservation(Detections& det, int frame, int detPos, int startTimeStemp);

    Vector<double> non_lin_state_equation(Vector<double> x, double dt);
    Matrix<double> makeQ(Vector<double> x, double dt);
    Matrix<double> makeF(Vector<double> x, double dt);
    Matrix<double> makeH();
    Matrix<double> makeR();
    Matrix<double> makeW();
    Vector<double> makeMeasurement();

    void saveData(int i);

    //**************************************************************
    // getter methods for getting parameters
    //**************************************************************

    void getIdx(Vector<FrameInlier>& Idx);
    void getRotation(Vector<double>& rot);
    void getVelocity(Vector<double>& vel);
    void getAllX(Vector< Vector<double> >& allX );
    void getColHist(Volume<double>& colHist);
    void getColHists(Vector<Volume <double> >& colHists);
    void getStateCovMatrices(Vector<Matrix<double> >& stateCovMats);

    void turnRotations(Vector<double> &dest,Vector<double> &src);

protected:

    //**************************************************************
    // Storage of the computed data, needed for the further process
    //**************************************************************

    Vector< Vector<double> > m_allXnew;
    Vector<FrameInlier> m_Idx;
    Vector<double> m_Rot;
    Vector<double> m_Vel;
    Vector< Matrix <double> > m_CovMats;
    Vector< Volume <double> > m_colHists;
    Volume<double> m_colHist;

    Vector<double> m_yPos;

    // R - the observation covariance
    Matrix<double> m_R;
    bool m_Up;
    double m_inlierOri;
    double m_height;

    Vector<double> m_bbox;

};
#endif	/* _DENNIS_KALMAN_H */

