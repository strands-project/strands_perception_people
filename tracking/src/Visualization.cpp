/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */


#include "Visualization.h"
#include <math.h>

Visualization::Visualization()
{

}

Visualization::~Visualization()
{
}

void
Visualization::setHypoBBoxColor(Vector<unsigned char>& currHypoColor,  map<int, int, greater<int> >& assignedBBoxCol, Matrix<unsigned char>& colors,
                                map<int, int, greater<int> >& hypoLastSelForVis, int t, int hypoID)
{
    map<int, int>::iterator it;

    bool colorFlag = false;
    int colorPos;
    currHypoColor.fill(0.0);
    int timeRangeBeforDrop = 50;


    //***************************************************
    // Update last Selected
    //***************************************************

    it = hypoLastSelForVis.find(hypoID);
    if(it != hypoLastSelForVis.end())
    {
        (*it).second = t;
    }
    else
    {
        hypoLastSelForVis[hypoID] = t;
    }

    //****************************************************
    // Check if last selected is to far in the past
    //****************************************************
    for(it = hypoLastSelForVis.begin(); it != hypoLastSelForVis.end(); ++it)
    {
        if(t - ((*it).second) > timeRangeBeforDrop)
        {
            assignedBBoxCol.erase((*it).first);
        }
    }

    //***************************************************
    // Speed Up, check if the hypo got an assigned color
    //***************************************************

    it = assignedBBoxCol.find(hypoID);
    if (it != assignedBBoxCol.end())
    {
        colors.getRow((*it).second, currHypoColor);
        return;
    }

    //****************************************************
    // If there is no color assigned to a hypoID,
    // check before to assign a color for the current hypo
    // which colors are already assigned to certain hypos
    //****************************************************

    for(int i = 0; i < colors.y_size(); i++)
    {
        if(assignedBBoxCol.size() > 0)
        {
            for(it = assignedBBoxCol.begin(); it != assignedBBoxCol.end(); ++it)
            {
                if((*it).second == i)
                {
                    colorFlag = true;
                    break;
                }
            }

            if(colorFlag)
            {
                colorFlag = false;
                continue;

            }else
            {
                assignedBBoxCol[hypoID] = i;
                colors.getRow(i, currHypoColor);
                colorFlag = true;
                break;
            }
        }
        else
        {
            assignedBBoxCol[hypoID] = i;
            colors.getRow(i, currHypoColor);
            colorFlag = true;
            break;
        }
    }


    if (!colorFlag)
    {
        colorPos = int (rand() % colors.y_size()-1);
        assignedBBoxCol[hypoID] = colorPos;
        colors.getRow(colorPos, currHypoColor);
    }
}

//void Visualization::render_hypoId(Vector<double> p1, Vector<double> color, Camera cam, QImage& m_image, int Id)
//{
//    QPainter painter(&m_image);
//    QColor qColor;
//    qColor.setRgb(color(0), color(1), color(2));

//    QPen pen;
//    pen.setColor(qColor);
//    painter.setPen(pen);

//    QFont font;
//    font.setBold(true);
//    font.setPointSize(12);
//    painter.setFont(font);


//    Vector<double> P1;

//    cam.WorldToImage(p1, Globals::WORLD_SCALE, P1);

//    char str[20];

//    sprintf(str, "%d", Id);
//    painter.drawText(P1(0), P1(1)-10, str);
//}


#ifdef cim_v
Vector<double> render_circle(const Camera& cam, CImg<unsigned char>& image, Vector<double> c, Vector<double> t,
                             double r, const unsigned char* color, double height)
{
    double xc = c(0), yc = c(1), zc = c(2);
    Vector<double> p3d(xc+r,yc,zc);
    cam.ProjectToGP(p3d,1, p3d);


    Vector<double> p(2), lp(2), p2(2), lp2(2);
    cam.WorldToImage(p3d,1,lp);

    Vector<double> copyX =(p3d);
    Vector<double>vCamGPN = cam.get_GPN();
    vCamGPN *= height;
    Vector<double> p_t   = copyX + vCamGPN;
    cam.WorldToImage(p_t,1,lp2);

    Vector<double> p_min(lp), p_max(lp);
    Vector<double> p_min_t(lp2), p_max_t(lp2);





    for(double t = 0.1; t<=2*M_PI; t+=0.1)
    {

        p3d(0) = r * cos(t) + xc;
        p3d(2) = r * sin(t) + zc;
        cam.ProjectToGP(p3d,1, p3d);

        Vector<double> copyX =(p3d);
        Vector<double>vCamGPN = cam.get_GPN();
        vCamGPN *= height;
        Vector<double> p_t   = copyX + vCamGPN;


        cam.WorldToImage(p3d,1,p);
        cam.WorldToImage(p_t,1,p2);

        image.draw_line_1(lp(0),lp(1),p(0),p(1),color,2);
        image.draw_line_1(lp2(0),lp2(1),p2(0),p2(1),color,2);
        if(p_max(0) < p(0)) p_max = p;
        if(p_min(0) > p(0)) p_min = p;

        if(p_max_t(0) < p2(0)) p_max_t = p2;
        if(p_min_t(0) > p2(0)) p_min_t = p2;

        lp = p;
        lp2 = p2;
    }

    Vector<double> res(p_min(0), p_min(1), p_max(0), p_max(1));
    Vector<double> res2(p_min_t(0), p_min_t(1), p_max_t(0), p_max_t(1));
    res.append(res2);
    return res;
}

void render_cylinder(const Camera& cam, CImg<unsigned char>& image, Vector<double> bc,
                     Vector<double> tc, double h, double r, const unsigned char* color)
{
    Vector<double> p1,p2;

//    Vector<double> tc(bc(0),bc(1)-h,bc(2));
    cam.WorldToImage(bc,1,p1);
    cam.WorldToImage(tc,1,p2);
    image.draw_line_1(p1(0),p1(1),p2(0),p2(1),color,2);

    p1=render_circle(cam,image,bc,tc,r,color, h);
//    p2=render_circle(cam,image,tc,r,color);
    image.draw_line_1(p1(0),p1(1),p1(4),p1(5),color,2);
    image.draw_line_1(p1(2),p1(3),p1(6),p1(7),color,2);

}

void Visualization::render_hypos(Camera cam, int frame,  map<int, int, greater<int> >& assignedBBoxCol,
                                 map<int, int, greater<int> >& hypoLastSelForVis, Matrix<unsigned char>& colors, CImg<unsigned char>& image, double speed,
                                 double objMinVel, double objWidth, double objLength, double objHeight, int hypoID, Vector<Vector<double> > vvHypoTrajPts,
                                 double WORLD_SCALE, Vector<double> vHypoX, Vector<double> vHypoDir, Vector<unsigned char>& currHypoColor)
{


    if(vvHypoTrajPts.getSize() == 0) return;
    srand (time(NULL));
    currHypoColor.setSize(3);


    //*****************************************************************
    // Get Camera Parameter.
    //*****************************************************************
    Vector<double> vCamPos = cam.get_t();
    Vector<double> vCamVPN = cam.get_VPN();
//    Vector<double> vCamGP = cam.get_GP();
    Vector<double> vCamGPN = cam.get_GPN();

    //****************************************************************
    // Go through all Hypos and render
    //****************************************************************

    double height;
    Vector<double> vOrt(3);
    Vector<double> auxDir(3);
    Vector<double> auxOrt(3);

    Vector<double> dirSize;
    Vector<double> ortSize;
    Vector<double> gpnHeight;


    Matrix<double> p(3, 12);

    Vector<double> vX;
    Vector<double> copyX;
    Vector<double> copyGPN;
    Vector<double> copyPos;
    Vector<double> aux;

    Vector<double> p1;
    Vector<double> p2;

    Vector<int> vPVis(12, -1);

    setHypoBBoxColor(currHypoColor, assignedBBoxCol, colors, hypoLastSelForVis, frame, hypoID);


    if(!(speed > objMinVel))
    {
        vHypoDir.setSize(3, 0);
        vHypoDir(0) = 1.0;
    }

    height = objHeight;
    vOrt = vHypoDir;
    vOrt.cross(vCamGPN);

    vX = vHypoX;

    // project the object onto the ground plane

    cam.ProjectToGP(vX, WORLD_SCALE, vX);

    auxDir   = cross(vHypoDir, vCamGPN);
    vHypoDir = cross(vCamGPN, auxDir);

    auxOrt   = cross(vOrt, vCamGPN);
    vOrt     = cross(vCamGPN, auxOrt);

    //*************************************************************************
    // create a cuboid model for the object
    //*************************************************************************
    dirSize = vHypoDir;
    dirSize *= objLength/2.0;

    ortSize = vOrt;
    ortSize *= objWidth/2.0;

    gpnHeight = vCamGPN;
    gpnHeight *= height;

    copyX = vX + dirSize;
    copyX +=(ortSize);
    p.insertRow(copyX, 0);

    copyX = vX;
    copyX -=(dirSize);
    copyX +=(ortSize);
    p.insertRow(copyX, 1);

    copyX = vX + dirSize;
    copyX -=(ortSize);
    p.insertRow(copyX, 2);

    copyX = vX - dirSize;
    copyX -=(ortSize);
    p.insertRow(copyX, 3);

    copyX = vX + dirSize;
    copyX +=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 4);

    copyX = vX - dirSize;
    copyX +=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 5);

    copyX = vX + dirSize;
    copyX -=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 6);

    copyX = vX - dirSize;
    copyX -=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 7);

    gpnHeight *=(0.5);
    copyX =vX + gpnHeight;
    p.insertRow(copyX, 8);

    copyX +=(dirSize);
    p.insertRow(copyX, 9);

    aux = (copyX);
    copyX =(vX);
    copyGPN = vCamGPN;
    copyGPN *= 0.35*objHeight;
    copyX   = vX + copyGPN;
    copyGPN = aux;
    aux -=(copyX);
    aux *=(0.25);
    copyGPN -=(aux);
    p.insertRow(copyGPN, 10);

    p.getRow(9, aux);
    copyX =(vX);
    copyGPN = vCamGPN;
    copyGPN *= 0.65*height;
    copyX +=(copyGPN);
    copyGPN = aux;
    aux -=(copyX);
    aux *=(0.25);
    copyGPN -=(aux);
    p.insertRow(copyGPN, 11);

    //***********************************************************
    // Check if Object is visible
    //***********************************************************

    copyPos =vCamPos*WORLD_SCALE;
    copyPos -=(vX);

    copyX =(vX);
    copyX *=(1.0/WORLD_SCALE);

    Vector<Vector<double> > points(12);
    Vector<double> points_mean(3,0.0);
    for(int j = 0; j < 8; j++)
    {
        p.getRow(j, aux);
        points(j) = aux;
        points_mean(0) += p(0,j);
        points_mean(1) += p(1,j);
        points_mean(2) += p(2,j);
        aux *=(1.0/WORLD_SCALE);
        if(cam.isPointInFrontOfCam(aux))
        {
            vPVis(j) = 1;
        }
    }
    points_mean*=1.0/8.0;

    for(int j = 8; j < 12; j++)
    {
        p.getRow(j, aux);
        points(j) = aux;
        aux *=(1.0/WORLD_SCALE);
        if(cam.isPointInFrontOfCam(aux))
        {
            vPVis(j) = 1;
        }
    }
    //***********************************************************
    // render the bbox lines on the screen.
    //***********************************************************

    char hpid[20];
    sprintf(hpid,"%d",hypoID);
    Vector<double> p2d;
    cam.WorldToImage(points_mean,WORLD_SCALE,p2d);
    if(Globals::render_tracking_numbers)
    {
        unsigned char bcolor[] = {0,0,0};
        image.draw_text(p2d(0),p2d(1),hpid,currHypoColor.data(),bcolor,1,30);
    }

    Vector<double> p_b = vX;

    copyX =(vX);
    copyGPN = vCamGPN;
    copyGPN *= height;
    Vector<double> p_t   = vX + copyGPN;

//    image.draw_line_1(p2d(0),p2d(1),xb,yb,currHypoColor.data(),2);
    render_cylinder(cam, image,p_b, p_t, height, objWidth/2, currHypoColor.data());

//    if(vPVis(0) & vPVis(1))
//    {
//        render_lines(cam, points(0), points(1), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(1) & vPVis(3))
//    {
//        render_lines(cam, points(1), points(3), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(3) & vPVis(2))
//    {
//        render_lines(cam, points(3), points(2), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(2) & vPVis(0))
//    {
//        render_lines(cam, points(2), points(0), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(4) & vPVis(5))
//    {
//        render_lines(cam, points(4), points(5), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(5) & vPVis(7))
//    {
//        render_lines(cam, points(5), points(7), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(7) & vPVis(6))
//    {
//        render_lines(cam, points(7), points(6), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(6) & vPVis(4))
//    {
//        render_lines(cam, points(6), points(4), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(0) & vPVis(4))
//    {
//        render_lines(cam, points(0), points(4), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(1) & vPVis(5))
//    {
//        render_lines(cam, points(1), points(5), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(2) & vPVis(6))
//    {
//        render_lines(cam, points(2), points(6), currHypoColor, image, WORLD_SCALE);
//    }

//    if(vPVis(3) & vPVis(7))
//    {
//        render_lines(cam, points(3), points(7), currHypoColor, image, WORLD_SCALE);
//    }

    if(speed > objMinVel)
    {
        if(vPVis(8) & vPVis(9))
        {
            p.getRow(8, p1);
            p.getRow(9, p2);
            render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
        }

        if(vPVis(9) & vPVis(10))
        {
            p.getRow(9, p1);
            p.getRow(10, p2);
            render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
        }

        if(vPVis(9) & vPVis(11))
        {
            p.getRow(9, p1);
            p.getRow(11, p2);
            render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
        }
    }

    //*************************************************************
    // Render trajectory
    //*************************************************************

    if(vvHypoTrajPts.getSize() > 1 && speed > objMinVel)
    {
        /////////////////////////////////////////
        // FIXME
        /////////////////////////////////////////
        smoothTraj(vvHypoTrajPts, 12);

        for(int j = 1; j < vvHypoTrajPts.getSize(); j++)
        {

            p1 = vvHypoTrajPts(j-1);
            p2 = vvHypoTrajPts(j);

            cam.ProjectToGP(p1, WORLD_SCALE, p1);
            cam.ProjectToGP(p2, WORLD_SCALE, p2);

            p1 *=(1.0/WORLD_SCALE);
            p2 *=(1.0/WORLD_SCALE);

            vCamVPN = cam.get_VPN();

            if(cam.isPointInFrontOfCam(p1) &&  cam.isPointInFrontOfCam(p2))
            {
                p1 *=(WORLD_SCALE);
                p2 *=(WORLD_SCALE);

                render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);

            }
        }

        //        cout << "8888888888888888888888888888888888888" << endl;
        //        cout << "vx " << vX(0) << " " << vX(1) << " " << vX(2) << endl;
        //        show_vec_vec(vvHypoTrajPts);

        //        cout << "8888888888888888888888888888888888888" << endl;

    }

    // FOR DAGM RENDER TRUNK POINT
    //                Vector<double> color;
    //                color.pushBack(255);
    //                color.pushBack(0);
    //                color.pushBack(0);
    //
    //                render_point(cam, image, trunkPoint, color, 8.0, WORLD_SCALE);
    //
    //                if(bbox.getSize() == 4)
    //                {
    //                    render_bbox_2D(bbox, image, 255, 255, 255, 1);
    //                }
}
#else
void Visualization::render_hypos(Camera cam, int frame,  map<int, int, greater<int> >& assignedBBoxCol,
                                 map<int, int, greater<int> >& hypoLastSelForVis, Matrix<int>& colors, QImage& image, double speed,
                                 double objMinVel, double objWidth, double objLength, double objHeight, int hypoID, Vector<Vector<double> > vvHypoTrajPts,
                                 double WORLD_SCALE, Vector<double> vHypoX, Vector<double> vHypoDir,
                                 Vector<double> /*bbox*/, Vector<int>& currHypoColor)
{


    if(vvHypoTrajPts.getSize() == 0) return;
    srand (time(NULL));
    currHypoColor.setSize(3);


    //*****************************************************************
    // Get Camera Parameter.
    //*****************************************************************
    Vector<double> vCamPos = cam.get_t();
    Vector<double> vCamVPN = cam.get_VPN();
    Vector<double> vCamGP = cam.get_GP();
    Vector<double> vCamGPN = cam.get_GPN();

    //****************************************************************
    // Go through all Hypos and render
    //****************************************************************

    double height;
    Vector<double> vOrt(3);
    Vector<double> auxDir(3);
    Vector<double> auxOrt(3);

    Vector<double> dirSize;
    Vector<double> ortSize;
    Vector<double> gpnHeight;


    Matrix<double> p(3, 12);

    Vector<double> vX;
    Vector<double> copyX;
    Vector<double> copyGPN;
    Vector<double> copyPos;
    Vector<double> aux;

    Vector<double> p1;
    Vector<double> p2;

    Vector<int> vPVis(12, -1);

    setHypoBBoxColor(currHypoColor, assignedBBoxCol, colors, hypoLastSelForVis, frame, hypoID);


    if(!(speed > objMinVel))
    {
        vHypoDir.setSize(3, 0);
        vHypoDir(0) = 1.0;
    }

    height = objHeight;
    vOrt = vHypoDir;
    vOrt.cross(vCamGPN);

    vX = vHypoX;

    // project the object onto the ground plane

    cam.ProjectToGP(vX, WORLD_SCALE, vX);

    auxDir   = cross(vHypoDir, vCamGPN);
    vHypoDir = cross(vCamGPN, auxDir);

    auxOrt   = cross(vOrt, vCamGPN);
    vOrt     = cross(vCamGPN, auxOrt);

    //*************************************************************************
    // create a cuboid model for the object
    //*************************************************************************
    dirSize = vHypoDir;
    dirSize *= objLength/2.0;

    ortSize = vOrt;
    ortSize *= objWidth/2.0;

    gpnHeight = vCamGPN;
    gpnHeight *= height;

    copyX = vX + dirSize;
    copyX +=(ortSize);
    p.insertRow(copyX, 0);

    copyX = vX;
    copyX -=(dirSize);
    copyX +=(ortSize);
    p.insertRow(copyX, 1);

    copyX = vX + dirSize;
    copyX -=(ortSize);
    p.insertRow(copyX, 2);

    copyX = vX - dirSize;
    copyX -=(ortSize);
    p.insertRow(copyX, 3);

    copyX = vX + dirSize;
    copyX +=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 4);

    copyX = vX - dirSize;
    copyX +=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 5);

    copyX = vX + dirSize;
    copyX -=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 6);

    copyX = vX - dirSize;
    copyX -=(ortSize);
    copyX +=(gpnHeight);
    p.insertRow(copyX, 7);

    gpnHeight *=(0.5);
    copyX =vX + gpnHeight;
    p.insertRow(copyX, 8);

    copyX +=(dirSize);
    p.insertRow(copyX, 9);

    aux = (copyX);
    copyX =(vX);
    copyGPN = vCamGPN;
    copyGPN *= 0.35*objHeight;
    copyX   = vX + copyGPN;
    copyGPN = aux;
    aux -=(copyX);
    aux *=(0.25);
    copyGPN -=(aux);
    p.insertRow(copyGPN, 10);

    p.getRow(9, aux);
    copyX =(vX);
    copyGPN = vCamGPN;
    copyGPN *= 0.65*height;
    copyX +=(copyGPN);
    copyGPN = aux;
    aux -=(copyX);
    aux *=(0.25);
    copyGPN -=(aux);
    p.insertRow(copyGPN, 11);

    //***********************************************************
    // Check if Object is visible
    //***********************************************************

    copyPos =vCamPos*WORLD_SCALE;
    copyPos -=(vX);

    copyX =(vX);
    copyX *=(1.0/WORLD_SCALE);

    //        if(cam.isPointInFrontOfCam(vX))
    //        {
    for(int j = 0; j < 12; j++)
    {
        p.getRow(j, aux);
        aux *=(1.0/WORLD_SCALE);
        if(cam.isPointInFrontOfCam(aux))
        {
            vPVis(j) = 1;
        }
    }

    //***********************************************************
    // render the bbox lines on the screen.
    //***********************************************************

    if(vPVis(0) & vPVis(1))
    {
        p.getRow(0, p1);
        p.getRow(1, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(1) & vPVis(3))
    {
        p.getRow(1, p1);
        p.getRow(3, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(3) & vPVis(2))
    {
        p.getRow(3, p1);
        p.getRow(2, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(2) & vPVis(0))
    {
        p.getRow(2, p1);
        p.getRow(0, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(4) & vPVis(5))
    {
        p.getRow(4, p1);
        p.getRow(5, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(5) & vPVis(7))
    {
        p.getRow(5, p1);
        p.getRow(7, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(7) & vPVis(6))
    {
        p.getRow(7, p1);
        p.getRow(6, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(6) & vPVis(4))
    {
        p.getRow(6, p1);
        p.getRow(4, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);

        //        render_hypoId(p1, currHypoColor, cam, image, id);
    }

    if(vPVis(0) & vPVis(4))
    {
        p.getRow(0, p1);
        p.getRow(4, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(1) & vPVis(5))
    {
        p.getRow(1, p1);
        p.getRow(5, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(2) & vPVis(6))
    {
        p.getRow(2, p1);
        p.getRow(6, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(vPVis(3) & vPVis(7))
    {
        p.getRow(3, p1);
        p.getRow(7, p2);
        render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
    }

    if(speed > objMinVel)
    {
        if(vPVis(8) & vPVis(9))
        {
            p.getRow(8, p1);
            p.getRow(9, p2);
            render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
        }

        if(vPVis(9) & vPVis(10))
        {
            p.getRow(9, p1);
            p.getRow(10, p2);
            render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
        }

        if(vPVis(9) & vPVis(11))
        {
            p.getRow(9, p1);
            p.getRow(11, p2);
            render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);
        }
    }

    //*************************************************************
    // Render trajectory
    //*************************************************************

    if(vvHypoTrajPts.getSize() > 1 && speed > objMinVel)
    {
        /////////////////////////////////////////
        // FIXME
        /////////////////////////////////////////
        smoothTraj(vvHypoTrajPts, 12);

        for(int j = 1; j < vvHypoTrajPts.getSize(); j++)
        {

            p1 = vvHypoTrajPts(j-1);
            p2 = vvHypoTrajPts(j);

            cam.ProjectToGP(p1, WORLD_SCALE, p1);
            cam.ProjectToGP(p2, WORLD_SCALE, p2);

            p1 *=(1.0/WORLD_SCALE);
            p2 *=(1.0/WORLD_SCALE);

            vCamVPN = cam.get_VPN();

            if(cam.isPointInFrontOfCam(p1) &&  cam.isPointInFrontOfCam(p2))
            {
                p1 *=(WORLD_SCALE);
                p2 *=(WORLD_SCALE);

                render_lines(cam, p1, p2, currHypoColor, image, WORLD_SCALE);

            }
        }

        //        cout << "8888888888888888888888888888888888888" << endl;
        //        cout << "vx " << vX(0) << " " << vX(1) << " " << vX(2) << endl;
        //        show_vec_vec(vvHypoTrajPts);

        //        cout << "8888888888888888888888888888888888888" << endl;

    }

    // FOR DAGM RENDER TRUNK POINT
    //                Vector<double> color;
    //                color.pushBack(255);
    //                color.pushBack(0);
    //                color.pushBack(0);
    //
    //                render_point(cam, image, trunkPoint, color, 8.0, WORLD_SCALE);
    //
    //                if(bbox.getSize() == 4)
    //                {
    //                    render_bbox_2D(bbox, image, 255, 255, 255, 1);
    //                }
}
#endif

//void Visualization::render_point(Camera cam, QImage& m_image, Vector<double> p3d, Vector<double> col, double size, double WORLD_SCALE)
//{
//    QPainter painter(&m_image);
//    QColor qColor;
//    qColor.setRgb(col(0), col(1), col(2));

//    QPen pen;
//    pen.setColor(qColor);
//    pen.setWidth(size);

//    painter.setPen(pen);

//    Vector<double> P1;

//    cam.WorldToImage(p3d, WORLD_SCALE, P1);

//    painter.drawPoint(P1(0), P1(1));
//}

#ifdef cim_v
void Visualization::render_lines(Camera cam, Vector<double>& p1, Vector<double>& p2, Vector<unsigned char> &color, CImg<unsigned char>& m_image, double WORLD_SCALE)
{
    //    QPainter painter(&m_image);
    int width = m_image.width()-3;//m_image.width()-3;
    int height = m_image.height()-3;//m_image.height()-3;

    //    QColor qColor;
    //    qColor.setRgb(color(0), color(1), color(2));

    //    QPen pen;
    //    pen.setColor(qColor);
    //    pen.setWidth(2);

    //    painter.setPen(pen);

    Vector<double> P1;
    Vector<double> P2;

    cam.WorldToImage(p1, WORLD_SCALE, P1);
    cam.WorldToImage(p2, WORLD_SCALE, P2);

    //*****************************************
    // create line
    //*****************************************

    Vector<double> o(2,0.0);
    Vector<double> r(2,0.0);
    Vector<double> res(2,0.0);

    o(0) = P1(0);
    o(1) = P1(1);
    r = o;
    r(0) = r(0) - P2(0);
    r(1) = r(1) - P2(1);

    //*******************************************
    // Left Side of the image
    //*******************************************
    Vector<double> oL(2,0.0);
    Vector<double> rL(2,0.0);
    rL(1) = -height;
    //*******************************************
    // Bottom Side of the image
    //*******************************************
    Vector<double> oB(2,0.0);
    oB(1) = height;
    Vector<double> rB(2,0.0);
    rB(0) = -width;
    //*******************************************
    // Upper Side of the image
    //*******************************************
    Vector<double> oU(2,0.0);
    Vector<double> rU(2,0.0);
    rU(0) = -width;
    //*******************************************
    // Right Side of the image
    //*******************************************
    Vector<double> oR(2,0.0);
    oR(0) = width;
    Vector<double> rR(2,0.0);
    rR(1) = -height;


    bool point1Out = false;
    bool point2Out = false;

    if(P1(0) >= width)
    {
        Math::crossLine(o, r, oR, rR, res);point1Out = true;
    }
    if(P1(0) < 0)
    {
        Math::crossLine(o, r, oL, rL, res);point1Out = true;
    }

    if(P1(1) >= height)
    {
        Math::crossLine(o, r, oB, rB, res);point1Out = true;
    }
    if(P1(1) < 0)
    {
        Math::crossLine(o, r, oU, rU, res);point1Out = true;
    }

    if(P2(0) >= width)
    {
        Math::crossLine(o, r, oR, rR, res);point2Out = true;
    }
    if(P2(0) < 0)
    {
        Math::crossLine(o, r, oL, rL, res);point2Out = true;
    }
    if(P2(1) >= height)
    {
        Math::crossLine(o, r, oB, rB, res);point2Out = true;
    }
    if(P2(1) < 0)
    {
        Math::crossLine(o, r, oU, rU, res);point2Out = true;
    }

    if((point1Out && point2Out))
    {
        return;
    }
    else if(point1Out)
    {
        //        painter.drawLine(res(0), res(1), P2(0), P2(1));
        m_image.draw_line_1(res(0), res(1), P2(0), P2(1),color.data(),2);

    }else if(point2Out)
    {
        //        painter.drawLine(P1(0), P1(1), res(0), res(1));
        m_image.draw_line_1(P1(0), P1(1), res(0), res(1),color.data(),2);
    }else
    {
        //        painter.drawLine(P1(0), P1(1), P2(0), P2(1));
        m_image.draw_line_1(P1(0), P1(1), P2(0), P2(1),color.data(),2);
    }


}
#else
void Visualization::render_lines(Camera cam, Vector<double>& p1, Vector<double>& p2, Vector<int>& color, QImage& m_image, double WORLD_SCALE)
{
    QPainter painter(&m_image);
    int width = m_image.width()-3;
    int height = m_image.height()-3;

    QColor qColor;
    qColor.setRgb(color(0), color(1), color(2));

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(2);

    painter.setPen(pen);

    Vector<double> P1;
    Vector<double> P2;

    cam.WorldToImage(p1, WORLD_SCALE, P1);
    cam.WorldToImage(p2, WORLD_SCALE, P2);

    //*****************************************
    // create line
    //*****************************************

    Vector<double> o(2,0.0);
    Vector<double> r(2,0.0);
    Vector<double> res(2,0.0);

    o(0) = P1(0);
    o(1) = P1(1);
    r = o;
    r(0) = r(0) - P2(0);
    r(1) = r(1) - P2(1);

    //*******************************************
    // Left Side of the image
    //*******************************************
    Vector<double> oL(2,0.0);
    Vector<double> rL(2,0.0);
    rL(1) = -height;
    //*******************************************
    // Bottom Side of the image
    //*******************************************
    Vector<double> oB(2,0.0);
    oB(1) = height;
    Vector<double> rB(2,0.0);
    rB(0) = -width;
    //*******************************************
    // Upper Side of the image
    //*******************************************
    Vector<double> oU(2,0.0);
    Vector<double> rU(2,0.0);
    rU(0) = -width;
    //*******************************************
    // Right Side of the image
    //*******************************************
    Vector<double> oR(2,0.0);
    oR(0) = width;
    Vector<double> rR(2,0.0);
    rR(1) = -height;


    bool point1Out = false;
    bool point2Out = false;

    if(P1(0) >= width)
    {
        Math::crossLine(o, r, oR, rR, res);point1Out = true;
    }
    if(P1(0) < 0)
    {
        Math::crossLine(o, r, oL, rL, res);point1Out = true;
    }

    if(P1(1) >= height)
    {
        Math::crossLine(o, r, oB, rB, res);point1Out = true;
    }
    if(P1(1) < 0)
    {
        Math::crossLine(o, r, oU, rU, res);point1Out = true;
    }

    if(P2(0) >= width)
    {
        Math::crossLine(o, r, oR, rR, res);point2Out = true;
    }
    if(P2(0) < 0)
    {
        Math::crossLine(o, r, oL, rL, res);point2Out = true;
    }
    if(P2(1) >= height)
    {
        Math::crossLine(o, r, oB, rB, res);point2Out = true;
    }
    if(P2(1) < 0)
    {
        Math::crossLine(o, r, oU, rU, res);point2Out = true;
    }

    if((point1Out && point2Out))
    {
        return;
    }
    else if(point1Out)
    {
        painter.drawLine(res(0), res(1), P2(0), P2(1));
    }else if(point2Out)
    {
        painter.drawLine(P1(0), P1(1), res(0), res(1));
    }else
    {
        painter.drawLine(P1(0), P1(1), P2(0), P2(1));
    }


}
#endif

#ifdef cim_v
void Visualization::render_bbox_2D(Vector<double> bbox, CImg<unsigned char>& image, int r, int g, int b, int lineWidth)
{
    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    unsigned char color[] = {r, g, b};
    image.draw_rectangle_1(x,y,x+w,y+h,color, 1, lineWidth);
}
#else
void Visualization::render_bbox_2D(Vector<double> bbox, QImage& image, int r, int g, int b, int lineWidth)
{

    QPainter painter(&image);

    QColor qColor;
    qColor.setRgb(r, g, b);

    QPen pen;
    pen.setColor(qColor);
    pen.setWidth(lineWidth);

    painter.setPen(pen);

    int x =(int) bbox(0);
    int y =(int) bbox(1);
    int w =(int) bbox(2);
    int h =(int) bbox(3);

    painter.drawLine(x,y, x+w,y);
    painter.drawLine(x,y, x,y+h);
    painter.drawLine(x+w,y, x+w,y+h);
    painter.drawLine(x,y+h, x+w,y+h);

    qColor.setRgb(0, 0, 0);
    pen.setColor(qColor);

    QRect rect = QRect(x, y, w, h);
    painter.drawText( rect, Qt::AlignLeft, "" );


}
#endif

void Visualization::smoothTraj(Vector<Vector<double> >& TrajPts, int nSmoothSize)
{
    Matrix<double> allPoints(TrajPts);

    int width = floor(nSmoothSize/2.0);
    int nrX = allPoints.y_size();

    Matrix<double> partMat;
    Matrix<double> smoothedPos(allPoints.x_size(), allPoints.y_size());
    Vector<double> mean;

    // ***********************************************************************
    //   Smooth the position of the trajectory
    // ***********************************************************************

    for(int i = 0; i < nrX; i++)
    {
        int cw = min(min(i, width),nrX-i-1);
        int upperCol = i - cw;
        int lowerCol = i + cw;

        allPoints.cutPart(0, allPoints.x_size()-1, upperCol, lowerCol, partMat);
        partMat.sumAlongAxisX(mean);

        if((lowerCol - upperCol) > 0)
        {
            mean *=(1.0/((lowerCol - upperCol) + 1));
        }
        smoothedPos.insertRow(mean, i);
    }

    TrajPts.clearContent();
    smoothedPos.transferMatToVecVec(TrajPts);
}
