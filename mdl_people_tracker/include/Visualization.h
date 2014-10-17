/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _DENNIS_VISUALIZATION_H
#define	_DENNIS_VISUALIZATION_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <map>
#include "Camera.h"
#include <algorithm>
#include "Math.h"
#include "Globals.h"
#include "CImageHeader.h"

#ifndef cim_v
#include <QImage>
#include <QString>
#include <QFileDialog>
#include <QRgb>
#include <QPainter>
#include <QColor>
#endif

using namespace std;


class Visualization
{
public:
    Visualization();
    ~Visualization();


#ifdef cim_v
    void render_hypos(Camera cam, int frame,  map<int, int, greater<int> >& assignedBBoxCol,
                      map<int, int, greater<int> >& hypoLastSelForVis, Matrix<unsigned char> &colors, CImg<unsigned char>& image, double speed,
                      double objMinVel, double objWidth, double objLength, double objHeight, int hypoID, Vector<Vector<double> > vvHypoTrajPts,
                      double WORLD_SCALE, Vector<double> vHypoX, Vector<double> vHypoDir, Vector<unsigned char> &currHypoColor);
    void render_bbox_2D(Vector<double> bbox, CImg<unsigned char>& image, int r, int g, int b, int lineWidth);
    void render_lines(Camera cam, Vector<double>& p1, Vector<double>& p2, Vector<unsigned char> &color, CImg<unsigned char>& image, double WORLD_SCALE);
#else
    void render_hypos(Camera cam, int frame,  map<int, int, greater<int> >& assignedBBoxCol,
                      map<int, int, greater<int> >& hypoLastSelForVis, Matrix<int> &colors, QImage& image, double speed,
                      double objMinVel, double objWidth, double objLength, double objHeight, int hypoID, Vector<Vector<double> > vvHypoTrajPts,
                      double WORLD_SCALE, Vector<double> vHypoX, Vector<double> vHypoDir, Vector<double> bbox, Vector<int> &currHypoColor);
    void render_bbox_2D(Vector<double> bbox, QImage& image, int r, int g, int b, int lineWidth);
    void render_lines(Camera cam, Vector<double>& p1, Vector<double>& p2, Vector<int> &color, QImage& image, double WORLD_SCALE);
#endif

    void smoothTraj(Vector<Vector<double> >& TrajPts, int nSmoothSize);
//    void render_point(Camera cam, QImage& m_image, Vector<double> p3d, Vector<double> col, double size, double WORLD_SCALE);
//    void render_hypoId(Vector<double> p1, Vector<double> color, Camera cam, QImage &image, int Id);


private:
    void setHypoBBoxColor(Vector<unsigned char> &currHypoColor,  map<int, int, greater<int> >& assignedBBoxCol, Matrix<unsigned char> &colors,
                          map<int, int, greater<int> >& hypoLastSelForVis, int t, int hypoID);


};



#endif		/* _DENNIS_VISUALIZATION_H */
