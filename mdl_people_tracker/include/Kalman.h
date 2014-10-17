/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

//#include <QObject>
#include "Matrix.h"

/***********************************************/
/* y   - the observation at time t             */
/* H - jacobian Observation Model              */
/* F - jacobian Dynamic Model                  */
/* Q - the system covariance                   */
/* R - the observation covariance              */
/* x - the state (column) Vector               */
/* P - the state covariance                    */
/***********************************************/

#ifndef _KALMANANGLE_H
#define	_KALMANANGLE_H


class Kalman
{
public:

    virtual void predict();
    virtual void update();
    virtual void init(Vector<double> xInit, Matrix<double> Pinit, double dt);

    virtual Vector<double> non_lin_state_equation(Vector<double> x, double dt) = 0;
    virtual Matrix<double> makeQ(Vector<double> x, double dt) = 0;
    virtual Matrix<double> makeF(Vector<double> x, double dt) = 0;
    virtual Matrix<double> makeH() = 0;
    virtual Matrix<double> makeR() = 0;
    virtual Matrix<double> makeW() = 0;
    virtual Vector<double> makeMeasurement() = 0;


    void showDeg(double a);

    Vector<double> m_xprio;
    Vector<double> m_xpost;

    Matrix<double> m_Pprio;
    Matrix<double> m_Ppost;

    Vector<double> m_measurement;

    double m_dt;

    bool m_measurement_found;

private:
        Vector<double> create3VecPos_Ori(Vector<double> vX);
        Vector<double> angleMinus(Vector<double> &a, Vector<double> &b);
};

#endif
