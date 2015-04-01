/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#include "Detections.h"
#include "AncillaryMethods.h"

using namespace std;

// ++++++++++++++++++++++++ constructor ++++++++++++++++++++++++++++++++++++++
Detections::Detections(int x, const int flag)
{
//    sizeOfDetVec = x;
    if (x == 22) {
        offSet = 0;
    } else {
        offSet = 1;
    }

    img_num = 0;
    hypo_num = 1;
    center_x = 2;
    center_y = 3;
    scale = 4;
    categ = 5;
    bbox = 5 + offSet; //consists of 4 num
    initscore = 9 + offSet;
    score = 10 + offSet;
    dist = 11 + offSet;
    height = 12 + offSet;
    rot = 19 + offSet; //consists of 3 num

    if (flag == 1) {
        pos = 13 + offSet; //consists of 3 num
    } else {
        pos = 16 + offSet; //consists of 3 num
    }

    colHists.setSize(Globals::numberFrames + Globals::nOffset);
    cov3d.setSize(Globals::numberFrames+ Globals::nOffset);
    detC.setSize(Globals::numberFrames + Globals::nOffset);
//    gp.setSize(Globals::numberFrames + Globals::nOffset);
//    points3D_.setSize(Globals::numberFrames + Globals::nOffset);
//    occBins_.setSize(Globals::numberFrames + Globals::nOffset);

    //    if(Globals::WORLD_SCALE != 0.001)
    //    {
    //        camLCov = Camera("cameraL", 1.0, 1.0);
    //        camRCov = Camera("cameraR", 1.0, 1.0);
    //        Matrix<double> K = camRCov.getCameraInt();
    //        Matrix<double> R = camRCov.getCameraRot();
    //        Vector<double> p = camRCov.getCameraPos();
    //        Vector<double> gp = camRCov.getGP();

    //        p *= (0.001)*Globals::WORLD_SCALE;
    //        camRCov = Camera(K, R, p, gp);
    //    }
    //    else
    //    {
    double lK_array[] = {500.683300000, 0.000000000, 318.572500000,
                         0.000000000, 500.252570000, 247.114520000,
                         0.000000000, 0.000000000, 1.000000000};
    Matrix<double> lK(3,3, lK_array);

    Matrix<double> lR; lR = Eye<double>(3);
    Vector<double> lp(3,0.0);
    Vector<double> lgp(0.00897055, -0.999531, -0.0292899, -964.274);
    camLCov = Camera(lK, lR, lp, lgp);

    double rK_array[] = {500.806960000, 0.000000000, 307.205730000,
                         0.000000000, 500.394060000, 233.520640000,
                         0.000000000, 0.000000000, 1.000000000};
    Matrix<double> rK(3,3, rK_array);

    double rR_array[] = {0.994584465, -0.008733820, -0.103563804,
                         0.008137584, 0.999947803, -0.006178313,
                         0.103612359, 0.005302095, 0.994603623};

    Matrix<double> rR(3,3, rR_array);
    Vector<double> rp(400.127285298, 2.724361699, 19.103868250);
    Vector<double> rgp(0.00897055, -0.999531, -0.0292899, -964.274);
    camRCov = Camera(rK, rR, rp, rgp);


    //    }
}



// ++++++++++++++++++++++++ Implementation ++++++++++++++++++++++++++++++++++++++

/*
 * Get sequence number from a detection. If the particular detection message was empty, return 0
 */
uint32_t Detections::getSeqNr(int frame, int detec){
    	//cout<<"trying to access frame: "<<frame<<" and detection: "<<detec<<endl;

    	Vector<Vector<double> > v=detC(frame);
    	int vs = v.getSize();

    	if(detec >= vs || frame >= detC.getSize()){
    		return 0;
    	}
    	return static_cast<unsigned int>(detC(frame)(detec)(24));
    }

/*
 * Get index from a detection. If the particular detection message was empty, return -1
 */
int Detections::getIndex(int frame, int detec){
	if(detec >= detC(frame).getSize() || frame >= detC.getSize()){
		return -1;
	}

	return static_cast<int>(detC(frame)(detec)(1));
}

int Detections::numberDetectionsAtFrame(int frame)
{
    if(frame >= detC.getSize())
    {
        return 0;
    }
    else
    {
        return detC(frame).getSize();
    }

}

double Detections::getHeight(int frame, int detec) {
    assert(detC.getSize() > frame);
    assert(detC(frame).getSize() > detec);

    return detC(frame)(detec)(height);
}

double Detections::getScore( int frame,  int detec) {
    assert(detC.getSize() > frame);
    assert(detC(frame).getSize() > detec);

    return detC(frame)(detec)(score);
}

void Detections::getPos3D( int frame,  int detec, Vector<double>& v_pos)
{
    v_pos.setSize(3);
    assert(detC.getSize() > frame);
    assert(detC(frame).getSize() > detec);

    v_pos(0) = (detC(frame)(detec)(pos));
    v_pos(1) = (detC(frame)(detec)(pos+1));
    v_pos(2) = (detC(frame)(detec)(pos+2));
}

void Detections::getBBox( int frame,  int detec, Vector<double>& v_bbox)
{
    v_bbox.clearContent();
    assert(detC.getSize() > frame);
    assert(detC(frame).getSize() > detec);

    v_bbox.pushBack(detC(frame)(detec)(bbox));
    v_bbox.pushBack(detC(frame)(detec)(bbox+1));
    v_bbox.pushBack(detC(frame)(detec)(bbox+2));
    v_bbox.pushBack(detC(frame)(detec)(bbox+3));
}

//int Detections::numberFrames ()
//{
//    return detC.getSize() ;
//}

Detections::~Detections()
{
    detC.clearContent();
    colHists.clearContent();
    cov3d.clearContent();
}

int Detections::prepareDet(Vector<double> &detContent, Vector<Vector <double> >& det,
                           int i/*, int frame*/, bool leftDet, Camera cam/*, Matrix<double>& depthMap*/, Matrix<double>& covariance)
{
    int img_num_hog = 0;
    int hypo_num_hog = 1;
    int scale_hog = 2;
    int score_hog = 3;
    int bbox_hog = 4;
    int distance_z = 8;

    detContent.setSize(25, 0.0);

    detContent(score) = det(i)(score_hog);
    detContent(scale) = det(i)(scale_hog) ;
    detContent(img_num) = det(i)(img_num_hog);
    detContent(hypo_num) = det(i)(hypo_num_hog);
    detContent(bbox) = floor(det(i)(bbox_hog) );
    detContent(bbox + 1) = floor(det(i)(bbox_hog + 1) );
    detContent(bbox + 2) = floor(det(i)(bbox_hog + 2) );
    detContent(bbox + 3) = floor(det(i)(bbox_hog + 3));
    if(leftDet)
    {
        detContent(22) = 0;
    }else
    {
        detContent(22) = 1;
    }

    // det is a vector, that contains the detection messages. Detection message contains the seq # that I want, at index 9
    // here, I'm saving the number
    detContent(24) = det(i)(9);


    Matrix<double> camRot = Eye<double>(3);
    Vector<double> camPos(3, 0.0);

    Vector<double> gp = cam.get_GP();

    Matrix<double> camInt = cam.get_K();

    Vector<double> planeInCam = projectPlaneToCam(gp, cam);
    Camera camI(camInt, camRot, camPos, planeInCam);

//    compute3DPosition(detContent, camI);

    Vector<double> posInCamCord(3,1.0);

    double c20 = camI.K()(2,0);
    double c00 = camI.K()(0,0);
    double c21 = camI.K()(2,1);
    double c11 = camI.K()(1,1);

    Vector<double> v_bbox(4);
    v_bbox(0) = (detContent(bbox));
    v_bbox(1) = (detContent(bbox+1));
    v_bbox(2) = (detContent(bbox+2));
    v_bbox(3) = (detContent(bbox+3)/3); // Use only upper body for histogram calculation

//            X[i] = z*(((i%width)-c20)/c00);
//            Y[i] = z*(((i/width)-c21)/c11);



            posInCamCord(0) = (det(i)(distance_z)*(v_bbox(0)+v_bbox(2)/2.0 - c20) / c00);
            posInCamCord(1) = (det(i)(distance_z)*(v_bbox(1) - c21) / c11);

    posInCamCord(2) = det(i)(distance_z);//+0.35; // Move from hull of cylinder to the center of mass of a pedestrian


    Vector<double>cp_posInCamCord = posInCamCord;
    camI.ProjectToGP(posInCamCord, 1, posInCamCord);

    cp_posInCamCord -= posInCamCord;
    detContent(height) = cp_posInCamCord.norm();

    posInCamCord.pushBack(1);






    if(posInCamCord(2) < 0)
    {
//        if(Globals::verbose)
//        {
//            cout << "Detection rejected due to inconsistency with DEPTH!!!" << endl;
//        }
        ROS_DEBUG("Detection rejected due to inconsistency with DEPTH!!!");
        return 0;
    }


    Vector<double> posInWorld = fromCamera2World(posInCamCord, cam);

    compute3DCov(posInCamCord, covariance);

    detContent(pos) = posInWorld(0);
    detContent(pos+1) = posInWorld(1);
    detContent(pos+2) = posInWorld(2);

    if(detContent(0) < 0) return 0;

    return 1;
}

Vector<double> Detections::projectPlaneToCam(Vector<double> p, Camera cam)
{
    Vector<double> gpInCam(4, 0.0);

    Vector<double> pv;
    pv.pushBack(p(0));
    pv.pushBack(p(1));
    pv.pushBack(p(2));

    Vector<double> camPos = cam.get_t();

    Matrix<double> camRot = cam.get_R();

    pv = Transpose(camRot)*pv;
    camRot *= -1.0;
    Vector<double> t = Transpose(camRot)*camPos;

    double d = p(3) - (pv(0)*t(0) + pv(1)*t(1) + pv(2)*t(2));

    gpInCam(0) = pv(0);
    gpInCam(1) = pv(1);
    gpInCam(2) = pv(2);
    gpInCam(3) = d;

    return gpInCam;
}

Vector<double> Detections::fromCamera2World(Vector<double> posInCamera, Camera cam)
{
    Matrix<double> rotMat = cam.get_R();

    Vector<double> posCam = cam.get_t();

    Matrix<double> trMat(4,4,0.0);
    trMat(3,3) = 1;
    trMat(0,0) = rotMat(0,0);
    trMat(0,1) = rotMat(0,1);
    trMat(0,2) = rotMat(0,2);
    trMat(1,0) = rotMat(1,0);
    trMat(1,1) = rotMat(1,1);
    trMat(1,2) = rotMat(1,2);
    trMat(2,0) = rotMat(2,0);
    trMat(2,1) = rotMat(2,1);
    trMat(2,2) = rotMat(2,2);

    posCam *= Globals::WORLD_SCALE;

    trMat(3,0) = posCam(0);
    trMat(3,1) = posCam(1);
    trMat(3,2) = posCam(2);

    Vector<double> transpoint = trMat*posInCamera;
    return transpoint;

}

double Detections::get_mediandepth_inradius(Vector<double>& bbox, int radius, Matrix<double>& depthMap, double var, double pOnGp)
{

    int middleX = max(radius,min((int)floor((bbox(0) + bbox(2)/2.0)), Globals::dImWidth-(radius + 1)));
    int middleY = max(min((int)floor((bbox(1) + bbox(3)/2.0)), Globals::dImHeight-(radius + 1)),radius);

    Vector<double> depthInRadius(2*radius*2*radius);
    int co = 0;
    for(int i = middleX - radius; i < middleX + radius; i++)
    {
        for(int j = middleY - radius; j < middleY + radius; j++)
        {
            depthInRadius(co) = (depthMap(i,j));
            co++;
        }
    }

    depthInRadius.sortV();
    double medianDepth = depthInRadius(floor(depthInRadius.getSize()/2.0));
    //     double meanDepth = depthInRadius(0);

    if(fabs(pOnGp-medianDepth) > var)
    {
        return -1;
    }
    else
    {
        return medianDepth;
    }
}

bool Detections::improvingBBoxAlignment_libelas(Vector<double>& vbbox, double var, Camera camera, Matrix<double>& depthMap)
{
    //======================================
    // Increase width and height of the bbox
    //======================================
    double incHeight = 0.2;

    vbbox(1) -= floor((double)(incHeight*vbbox(3))/2.0);
    vbbox(3) += floor(incHeight*vbbox(3));

    //================================================
    // Get Camera Parameters
    //================================================

    Matrix<double> intrinsic = camera.get_K();

    //================================================
    // Find the middle of the bbox and find a median
    // in radius of the middle.
    //================================================
    int k = 0;

    vbbox(1) = vbbox(1) < 0 ? 0 : vbbox(1);
    vbbox(0) = vbbox(0) < 0 ? 0 : vbbox(0);
    vbbox(3) = vbbox(1)+vbbox(3) >= Globals::dImHeight - 1 ? vbbox(3) - (vbbox(1)+vbbox(3) - Globals::dImHeight) - 1 : vbbox(3);
    vbbox(2) = vbbox(0)+vbbox(2) >= Globals::dImWidth - 1 ? vbbox(2) - (vbbox(0)+vbbox(2) - Globals::dImWidth) - 1 : vbbox(2);

    std::vector< double > medv(((vbbox(2)+1)/2) * ((vbbox(3)+1)/2));
    for (int y = vbbox(1); y < vbbox(1) + vbbox(3); y += 2) {
        for (int x = vbbox(0); x < vbbox(0) + vbbox(2); x += 2) {
            if (depthMap(x, y) == 0 && depthMap(x, y) == Globals::farPlane) {
                continue;
            }
            medv[k++] = depthMap(x, y);
        }
    }

    if(k < 10)
    {
        return false;
    }

    sort(medv.begin(), medv.begin() + k);
    double medianDepth = medv[k / 2];

    //================================================
    // Get pos in 3D in Camera coordinate
    //================================================

    Vector<double> pos2D;
    pos2D.pushBack(vbbox(0)+vbbox(2)/2.0);
    pos2D.pushBack(vbbox(1)+vbbox(3)/2.0);
    pos2D.pushBack(1);

    Vector<double> pos3D(3);
    pos3D(0) = medianDepth*((pos2D(0)-intrinsic(2,0))/intrinsic(0,0));
    pos3D(1) = medianDepth*((pos2D(1)-intrinsic(2,1))/intrinsic(1,1));
    pos3D(2) = medianDepth;

    //====================================================
    // Project the 3D Point on the ground plane
    //====================================================

    camera.ProjectToGP(pos3D, Globals::WORLD_SCALE, pos3D);

    //====================================================
    // Using the media depth find the head of the person
    // more exactly find the new Y position of the bbox
    //====================================================

    vbbox(1) = vbbox(1) < 2 ? 2 : vbbox(1);
    vbbox(0) = vbbox(0) < 2 ? 2 : vbbox(0);
    vbbox(3) = vbbox(1)+vbbox(3) >= Globals::dImHeight - 3 ? vbbox(3) - (vbbox(1)+vbbox(3) - Globals::dImHeight) - 3 : vbbox(3);
    vbbox(2) = vbbox(0)+vbbox(2) >= Globals::dImWidth - 3 ? vbbox(2) - (vbbox(0)+vbbox(2) - Globals::dImWidth) - 3 : vbbox(2);

    if(vbbox(1)+vbbox(3) < 0)
        return  false;

    if(vbbox(0)+vbbox(2) < 0)
        return false;

    int newYAbove = -1;
    int c = 5;

    int j = vbbox(0) + vbbox(2)/2.0;
    int i = vbbox(1);

    for(; i  < vbbox(1)+vbbox(3)/2.0; i++)
    {
        if (c == 0){break;}
        // Check also the neighboors
        if(depthMap(j,i) < medianDepth + var || depthMap(j-2,i) < medianDepth + var ||  depthMap(j+2,i) < medianDepth + var)
        {
            c -= 1;
            newYAbove = i;
        }
    }

    if(newYAbove < 0)
    {
        return false;
    }
    //=====================================================
    // As next step we will have to find the bottom pos in
    // 2D using the backprojection form 3D
    //=====================================================

    Vector<double> p1;
    camera.WorldToImage(pos3D, Globals::WORLD_SCALE, p1);

    int newYBottom = floor(p1(1));

    vbbox(1) = newYAbove-5;
    vbbox(3) = newYBottom - vbbox(1);

    return true;

}

#ifdef cim_v
void Detections::addHOGdetOneFrame(Vector<Vector <double> >& det, int frame, CImg<unsigned char>& imageLeft, Camera cam/*, Matrix<double>& depthMap*/)
{

    // FIXME find a differnt approach for determing a 3D cov.

    Matrix<double> covariance(3,3,0.0);

    Volume<double> colhist;
    Vector<double> pos3d(3);
    Vector<double> v_bbox(4);


    Vector<double>detContent;

    for ( int i = 0; i < det.getSize(); i++)
    {
        if(prepareDet(detContent, det, i, true, cam, covariance))
        {
            pos3d(0) = (detContent(pos));
            pos3d(1) = (detContent(pos+1));
            pos3d(2) = (detContent(pos+2));

            v_bbox(0) = (detContent(bbox));
            v_bbox(1) = (detContent(bbox+1));
            v_bbox(2) = (detContent(bbox+2));
            v_bbox(3) = (detContent(bbox+3)/3); // Use only upper body for histogram calculation

            computeColorHist(colhist, v_bbox, Globals::binSize, imageLeft);

            //ROS_FATAL_STREAM("2) seq number of frame "<< frame <<" saved in detContent:"<<detContent(24));
            detC(frame).pushBack(detContent); //HERE is the push of the single detection into the vector of all detections.


            colHists(frame).pushBack(colhist);

            covariance(0,0) = sqrtf(covariance(0,0));
            covariance(2,2) = sqrtf(covariance(2,2));
            cov3d(frame).pushBack(covariance);
        }
    }
}
#else
void Detections::addHOGdetOneFrame(Vector<Vector <double> >& det, int frame, QImage& imageLeft, Camera cam, Matrix<double>& depthMap)
{

    // FIXME find a differnt approach for determing a 3D cov.

    Matrix<double> covariance(3,3,0.0);

    Volume<double> colhist;
    Vector<double> pos3d;
    Vector<double> v_bbox;


    Vector<double>detContent;

    for ( int i = 0; i < det.getSize(); i++)
    {
        if(prepareDet(detContent, det, i, frame, true, cam, depthMap, covariance))
        {
            pos3d.clearContent();
            pos3d.pushBack(detContent(pos));
            pos3d.pushBack(detContent(pos+1));
            pos3d.pushBack(detContent(pos+2));

            v_bbox.clearContent();
            v_bbox.pushBack(detContent(bbox));
            v_bbox.pushBack(detContent(bbox+1));
            v_bbox.pushBack(detContent(bbox+2));
            v_bbox.pushBack(detContent(bbox+3));

            computeColorHist(colhist, v_bbox, Globals::binSize, imageLeft);

            detC(frame).pushBack(detContent);
            colHists(frame).pushBack(colhist);

            covariance(0,0) = sqrtf(covariance(0,0));
            covariance(2,2) = sqrtf(covariance(2,2));
            cov3d(frame).pushBack(covariance);
        }
    }
}
#endif

//void Detections::getDetection(int frame, int detec, Vector<double>& det)
//{
//    det = detC(frame)(detec);
//}

int Detections::getCategory(int frame, int detec)
{
    return detC(frame)(detec)(categ);
}

//int Detections::getDetNumber(int frame, int detec)
//{
//    return detC(frame)(detec)(hypo_num);
//}

//Vector<Vector<double> > Detections::get3Dpoints(int frame, int detec)
//{
//    return points3D_(frame)(detec);
//}

//Vector<Vector<int> > Detections::getOccCells(int frame, int detec)
//{
//    return occBins_(frame)(detec);
//}

void Detections::compute3DPosition(Vector<double>& detection, Camera cam)
{
    Vector<double> pos3D;
    Vector<double> v_bbox(4);
    double distance;

    v_bbox(0) = (detection(bbox));
    v_bbox(1) = (detection(bbox+1));
    v_bbox(2) = (detection(bbox+2));
    v_bbox(3) = (detection(bbox+3));

    double f_height = cam.bbToDetection(v_bbox, pos3D, Globals::WORLD_SCALE, distance);
    detection(dist) = distance;

    //*********************************************************************************
    // Having the 3D pos, postpone the footpoint to the middle of the BBOX in 3D
    //*********************************************************************************

    //    Vector<double> vpn = cam.get_VPN();
    //    Vector<double> gpn = cam.get_GPN();

    //    vpn *= Globals::pedSizeWVis / 2.0;

    //    Vector<double> t = cross(vpn, gpn);
    //    gpn.cross(t);

    //        pos3D += gpn;

    detection(pos) = pos3D(0);
    detection(pos+1) = pos3D(1);
    detection(pos+2) = pos3D(2);
    detection(height) =  f_height;

    //**********************************************************************************
    // Test Hard decision, if height doent reach some threshold dont use this detection
    //**********************************************************************************

    if(exp(-((Globals::dObjHeight - f_height)*(Globals::dObjHeight - f_height)) /
           (2 * Globals::dObjHVar * Globals::dObjHVar))  < Globals::probHeight || distance < 0)
    {
//        if(Globals::verbose)
//        {
//            cout << "Height Test Failed! " << f_height << endl;
//        }
        ROS_DEBUG("Height Test Failed! %f", f_height);
        detection(0) *= -1;
    }
}

#ifdef cim_v
void Detections::computeColorHist(Volume<double>& colHist, Vector<double>& bbox, int nBins, CImg<unsigned char>& m_image)
{
    colHist.setSize(nBins, nBins, nBins, 0.0);

    Matrix<double> mNorm;
    Vector<double> vNorm;

    int r = 0;
    int g = 0;
    int b = 0;


    double x = bbox[0];
    double y = bbox[1];
    double w = bbox[2];
    double h = bbox[3];

    double binSize = 256.0 / double(nBins);
    double weight;

    double a = 1.0/(w*Globals::cutWidthBBOXColor*w*Globals::cutWidthBBOXColor*0.6);
    double c = 1.0/(h*Globals::cutHeightBBOXforColor*h*Globals::cutHeightBBOXforColor*0.6);

    //*********************************************************
    // Parameter for the eliptic shape
    //*********************************************************

    double newHeight = floor(h - (h * Globals::cutHeightBBOXforColor));
    double newWidth = floor(w - (w * Globals::cutWidthBBOXColor));
    int centerEliX = floor(x + (w / 2.0));
    int centerEliY = floor(y + (newHeight / 2.0) + newHeight*Globals::posponeCenterBBOXColor);

    for(int i = x; i < x+w; i++)
    {
        for(int j = y; j < y+h; j++)
        {
            if(Math::evalElipse(newWidth, newHeight, centerEliX, centerEliY, i, j) && i < Globals::dImWidth && j < Globals::dImHeight && i > -1 && j > -1)
            {
                r = m_image(i,j,0,0);
                g = m_image(i,j,0,1);
                b = m_image(i,j,0,2);

                // Just for visualizing the area (must be removed)
//                m_image(i,j,0,0)=0;
//                m_image(i,j,0,1)=0;
//                m_image(i,j,0,2)=0;

                /////////////////////////////////////////////////

                r = floor(double(r)/binSize);
                g = floor(double(g)/binSize);
                b = floor(double(b)/binSize);

                weight = exp(-(a*(i-centerEliX)*(i-centerEliX) + c*(j - centerEliY)*(j - centerEliY)));
                colHist(r, g, b) += weight;

            }
        }
    }

    colHist.sumAlongAxisZ(mNorm);
    mNorm.sumAlongAxisX(vNorm);
    double number = vNorm.sum();
    colHist *= (1.0/number);
}
#else
void Detections::computeColorHist(Volume<double>& colHist, Vector<double>& bbox, int nBins, QImage& m_image) {

    colHist.setSize(nBins, nBins, nBins, 0.0);

    Matrix<double> mNorm;
    Vector<double> vNorm;

    int r = 0;
    int g = 0;
    int b = 0;

    QColor color(r, g, b);
    QRgb rgb = color.rgb();

    double x = bbox[0];
    double y = bbox[1];
    double w = bbox[2];
    double h = bbox[3];

    double binSize = 256.0 / double(nBins);
    double weight;

    double a = 1.0/(w*Globals::cutWidthBBOXColor*w*Globals::cutWidthBBOXColor*0.6);
    double c = 1.0/(h*Globals::cutHeightBBOXforColor*h*Globals::cutHeightBBOXforColor*0.6);

    //*********************************************************
    // Parameter for the eliptic shape
    //*********************************************************

    double newHeight = floor(h - (h * Globals::cutHeightBBOXforColor));
    double newWidth = floor(w - (w * Globals::cutWidthBBOXColor));
    int centerEliX = floor(x + (w / 2.0));
    int centerEliY = floor(y + (newHeight / 2.0) + newHeight*Globals::posponeCenterBBOXColor);

    for(int i = x; i < x+w; i++)
    {
        for(int j = y; j < y+h; j++)
        {
            if(Math::evalElipse(newWidth, newHeight, centerEliX, centerEliY, i, j) && i < Globals::dImWidth && j < Globals::dImHeight && i > -1 && j > -1)
            {
                rgb = m_image.pixel(i,j);
                r = qRed(rgb);
                g = qGreen(rgb);
                b = qBlue(rgb);

                r = floor(double(r)/binSize);
                g = floor(double(g)/binSize);
                b = floor(double(b)/binSize);

                weight = exp(-(a*(i-centerEliX)*(i-centerEliX) + c*(j - centerEliY)*(j - centerEliY)));
                colHist(r, g, b) += weight;

            }
        }
    }

    colHist.sumAlongAxisZ(mNorm);
    mNorm.sumAlongAxisX(vNorm);
    double number = vNorm.sum();
    colHist *= (1.0/number);
}
#endif

void Detections::getColorHist(int frame, int pos, Volume<double>& colHist)
{
    colHist = colHists(frame)(pos);
}

void Detections::compute3DCov(Vector<double> pos3d, Matrix<double>& cov)
{

    Matrix<double> covL;
    Matrix<double> covR;

    Matrix<double> c2d(2,2,0.0);
    c2d(0,0) = 0.5;
    c2d(1,1) = 0.5;

    c2d.inv();
    //****************************************************//
    // Computed as follows                                //
    // C = inv(F1' * inv(c2d) * F1 + F2' * inv(c2d) * F2);//
    //****************************************************//

    pos3d(0) *= (1.0 / Globals::WORLD_SCALE);
    pos3d(1) *= (1.0 / Globals::WORLD_SCALE);
    pos3d(2) *= (1.0 / Globals::WORLD_SCALE);

    camLCov.jacFor3DCov(pos3d, covL);
    camRCov.jacFor3DCov(pos3d, covR);

    Matrix<double> covLT = Transpose(covL);
    Matrix<double> covRT = Transpose(covR);

    covLT *= c2d;
    covLT *= covL;

    covRT *= c2d;
    covRT *= covR;

    covLT += covRT;
    covLT.inv();

    cov = covLT;
    cov *= (Globals::WORLD_SCALE)*(Globals::WORLD_SCALE);

    cov(0,0) = sqrt(cov(0,0));
    cov(2,2) = sqrt(cov(2,2));

//    cov.Show();

}

void Detections::get3Dcovmatrix(int frame, int pos, Matrix<double>& covariance)
{
    covariance = cov3d(frame)(pos);
}

//void Detections::setScore(int frame, int pos, double scoreValue)
//{
//    assert(frame < detC.getSize());
//    detC(frame)(pos)(score) = scoreValue;
//}

