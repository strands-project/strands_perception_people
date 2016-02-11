/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#include "Tracker.h"

using namespace std;


Tracker::Tracker()
{
    unsigned char color_array[] = {   204,     0,   255,
                                      255,     0,     0,
                                      0,   178,   255,
                                      255,     0,   191,
                                      255,   229,     0,
                                      0,   255,   102,
                                      89,   255,     0,
                                      128,     0,   255,
                                      242,     0,   255,
                                      242,   255,     0,
                                      255,     0,    77,
                                      51,     0,   255,
                                      0,   255,   140,
                                      0,   255,    25,
                                      204,   255,     0,
                                      255,   191,     0,
                                      89,     0,   255,
                                      0,   217,   255,
                                      0,    64,   255,
                                      255,   115,     0,
                                      255,     0,   115,
                                      166,     0,   255,
                                      13,     0,   255,
                                      0,    25,   255,
                                      0,   255,   217,
                                      0,   255,    64,
                                      255,    38,     0,
                                      255,     0,   153,
                                      0,   140,   255,
                                      255,    77,     0,
                                      255,   153,     0,
                                      0,   255,   179,
                                      0,   102,   255,
                                      255,     0,    38,
                                      13,   255,     0,
                                      166,   255,     0,
                                      0,   255,   255,
                                      128,   255,     0,
                                      255,     0,   230,
                                      51,   255,     0
                                  };

    possibleColors = Matrix<unsigned char>(3, 40, color_array);
    lastHypoID = -1;
//    aStream = new ofstream("export_bboxes.txt");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////                                               MONO                                                    /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double rand_doubleRange(double a, double b)
{
    return ((b-a)*((double)rand()/RAND_MAX))+a;
}

void Tracker::check_termination(Camera &cam, Vector<Hypo>& HyposAll)
{
    Vector<double> gp = cam.get_GP();

    if (HyposAll.getSize() != 0)
    {
        //*********************************************************
        // Define the "exit zones"
        //*********************************************************

        Vector<double> aux(3, 1);
        aux(0) = 1;
        aux(1) = Globals::dImHeight*(5.0/5.0);
        Vector<double> vXTL = AncillaryMethods::backprojectGP(aux, cam, gp);
        aux(0) = 1;
        aux(1) = Globals::dImHeight*(1.0/5.0);
        Vector<double> vXBL = AncillaryMethods::backprojectGP(aux, cam, gp);
        aux(0) = Globals::dImWidth - 1;
        aux(1) = Globals::dImHeight*(5.0/5.0);
        Vector<double> vXTR = AncillaryMethods::backprojectGP(aux, cam, gp);
        aux(0) = Globals::dImWidth - 1;
        aux(1) = Globals::dImHeight*(1.0/5.0);
        Vector<double> vXBR = AncillaryMethods::backprojectGP(aux, cam, gp);

        // Calculate exit planes, 4th value in normal just keeps time stamp

        Vector<double> Gp = cam.get_GPN();
        Vector<double> nR4d;
        Vector<double> nL4d;
        Vector<double> nC4d;
        double dL4d;
        double dR4d;
        double dC4d;

        //Gp *= 100.0;

        vXBL -= vXTL;
        nL4d = cross(Gp, vXBL);
        nL4d *= 1.0 / nL4d.norm();
        dL4d = -(DotProduct(nL4d, vXTL));

        vXBR -= vXTR;
        nR4d = cross(Gp, vXBR);
        nR4d *= -1.0;
        nR4d *= 1.0 / nR4d.norm();
        dR4d = -(DotProduct(nR4d, vXTR));

        // Principal plane of left camera as 'behind camera' exit zone

        Matrix<double> KRt = cam.get_KRt();
        Vector<double> KRtT = cam.get_KRtT();

        nC4d = KRt.getRow(2);
        dC4d = -KRtT(2) * Globals::WORLD_SCALE;

        //unsigned int nStartTolerance = 10;
        int nEndTolerance = 0;
        double zonesizeL = 0.7;
        double zonesizeR = 0.7;
        double zonesizeC = 1;
        double zonesizeC2 = 1.5;

        Vector<Vector <double> > TrajPts;



        Vector<int> hyposToRemove;


        for (int i = 0; i < HyposAll.getSize(); i++)
        {

            HyposAll(i).getTrajPts(TrajPts);

            Vector<double> dvec1;
            Vector<double> dvec2;
            Vector<double> dvec3;

            int j = TrajPts.getSize()-1;
            //            for(int j = 0; j < TrajPts.getSize(); j++)
            //            {
            dvec1.pushBack(TrajPts(j)(0) * nL4d(0) + TrajPts(j)(1)*nL4d(1) + TrajPts(j)(2)*nL4d(2) + dL4d);
            dvec2.pushBack(TrajPts(j)(0) * nR4d(0) + TrajPts(j)(1)*nR4d(1) + TrajPts(j)(2)*nR4d(2) + dR4d);
            dvec3.pushBack(TrajPts(j)(0) * nC4d(0) + TrajPts(j)(1)*nC4d(1) + TrajPts(j)(2)*nC4d(2) + dC4d);
            //            }

            Vector<int> IdxVec1;
            Vector<int> IdxVec2;
            Vector<int> IdxVec3;
            Vector<int> IdxVec4;

            for (int j = 0; j < dvec1.getSize(); j++)
            {
                if (dvec1(j) < zonesizeL) IdxVec1.pushBack(j);
                if (dvec2(j) < zonesizeR) IdxVec2.pushBack(j);
                if (dvec3(j) < zonesizeC) IdxVec3.pushBack(j);
                if (dvec3(j) < zonesizeC2) IdxVec4.pushBack(j);
            }


            //            if (IdxVec3.getSize() > nEndTolerance || (IdxVec4.getSize() > 2))
            //                        {
            //                            if(Globals::verbose){
            //                                cout << "HYPO entered entered behind camera EXIT ZONE" << endl;
            //                            }
            //                            hyposToRemove.pushBack(i);
            //                        }

            if(IdxVec1.getSize() > nEndTolerance)
            {
//                if(Globals::verbose){
//                    cout << "HYPO entered left EXIT ZONE" << endl;
//                }
                ROS_DEBUG("HYPO entered left EXIT ZONE");
                hyposToRemove.pushBack(i);
            }
            else if(IdxVec2.getSize() > nEndTolerance)
            {
//                if(Globals::verbose){
//                    cout << "HYPO entered right EXIT ZONE" << endl;
//                }
                ROS_DEBUG("HYPO entered right EXIT ZONE");
                hyposToRemove.pushBack(i);
            }else if (IdxVec3.getSize() > nEndTolerance || (IdxVec4.getSize() > 2))
            {
//                if(Globals::verbose){
//                    cout << "HYPO entered entered behind camera EXIT ZONE" << endl;
//                }
                ROS_DEBUG("HYPO entered entered behind camera EXIT ZONE");
                hyposToRemove.pushBack(i);
            }



        }
        Vector<Hypo> remainedHypos;
        for(int i = 0; i < HyposAll.getSize() ; i++)
        {
            if(hyposToRemove.findV(i) < 0)
            {
                remainedHypos.pushBack(HyposAll(i));
            }
        }

        HyposAll = remainedHypos;
    }
}

#ifndef cim_v
void Tracker::process_tracking_oneFrame(Vector<Hypo>& HyposAll, Detections& allDet, int frame,
                                        Vector<Vector<double> > foundDetInFrame, QImage& im, Camera cam, Matrix<double> depthMap)
{

    char imageSavePath[200];
    //    Vector<double> camPos = cam.get_t();

    allDet.addHOGdetOneFrame(foundDetInFrame, frame, im, cam, depthMap);

    //*****************************************************************************************************
    HyposMDL.clearContent();
    process_frame( allDet , cam, frame, HyposMDL, HyposAll, HypoEnded, hypoStack, possibleColors, assignedBBoxCol, hypoLastSelForVis);


    //***************************************************************************************
    // Visualization part 3D
    //***************************************************************************************
    Vector<Vector<double> > vvHypoTrajPts;
    Vector<double> vX(3);
    Vector<double> vDir;
    Vector<FrameInlier> Idx;

    Vector<double> bbox;
    Vector<int> colors;
    Visualization vis;

    int number_of_frames_hypo_visualized_without_inlier = 7;


    Vector<Vector<double> > hyposToWrite(HyposMDL.getSize());


    if(Globals::render_bbox3D)
    {

        for(int i = 0; i < HyposMDL.getSize(); i++)
        {

            //***************************************************************************************
            // Render only if the last occurence of an inlier is not far away
            //***************************************************************************************
            Vector<FrameInlier> inlier;
            HyposMDL(i).getIdx(inlier);

            hyposToWrite(i).setSize(5);

            bbox.clearContent();
            HyposMDL(i).getTrajPts(vvHypoTrajPts);
            Matrix<double> allP;
            HyposMDL(i).getXProj(allP);
            vX = allP.getRow(allP.y_size()-1);
            vX(2) = vX(1);
            vX(1) = vX(3);
            vX.resize(3);
            hyposToWrite(i)(0) = HyposMDL(i).getHypoID();
            hyposToWrite(i)(1) = vX(0);
            hyposToWrite(i)(2) = vX(1);
            hyposToWrite(i)(3) = vX(2);
            hyposToWrite(i)(4) = HyposMDL(i).getHeight();

            //            if(frame - inlier(inlier.getSize()-1).getFrame() > number_of_frames_hypo_visualized_without_inlier)
            //            {
            //                cout << "stop visualization of a Hypo due to missing inliers !!!" << endl;
            //                continue;
            //            }

            Vector<double> b(3);
            vvHypoTrajPts.clearContent();
            vvHypoTrajPts.setSize(allP.y_size());
            for(int j = 0; j < vvHypoTrajPts.getSize(); j++)
            {
                b(0) = allP(0,j);
                b(1) = allP(3,j);
                b(2) = allP(1,j);
                vvHypoTrajPts(j) = b;
            }

            HyposMDL(i).getDir(vDir);
            HyposMDL(i).getIdx(Idx);


            vis.render_hypos(cam, frame, assignedBBoxCol, hypoLastSelForVis, possibleColors, im, HyposMDL(i).getSpeed(), Globals::minvel, Globals::pedSizeWVis,
                             Globals::pedSizeWVis, HyposMDL(i).getHeight(), HyposMDL(i).getHypoID(), vvHypoTrajPts, Globals::WORLD_SCALE, vX, vDir, bbox, colors);


        }

        int nrDet = allDet.numberDetectionsAtFrame(frame);
        Vector<double> bbox;
        for(int i = 0; i < nrDet; i++)
        {
            allDet.getBBox(frame, i, bbox);
            vis.render_bbox_2D(bbox, im, 0, 255, 0, 1);
        }

    }

    AncillaryMethods::exportBBOX(HyposMDL, cam, frame, *aStream);

    //    sprintf(imageSavePath,  Globals::path_to_results_hypos.c_str(), frame);
    //    AncillaryMethods::write_vec_vec_to_disk(hyposToWrite,imageSavePath);

}
#else
void Tracker::process_tracking_oneFrame(Vector<Hypo>& HyposAll, Detections& allDet, int frame,
                                        Vector<Vector<double> > &foundDetInFrame, CImg<unsigned char>& im, Camera &cam/*, Matrix<double> &depthMap*/)
{
//    char imageSavePath[200];
//    Vector<double> camPos = cam.get_t();

    allDet.addHOGdetOneFrame(foundDetInFrame, frame, im, cam/*, depthMap*/);

    //*****************************************************************************************************
    HyposMDL.clearContent();
    process_frame( allDet , cam, frame, HyposAll);


    //***************************************************************************************
    // Visualization part 3D
    //***************************************************************************************
    Vector<Vector<double> > vvHypoTrajPts;
    Vector<double> vX(3);
    Vector<double> vDir;
//    Vector<FrameInlier> Idx;

    Vector<unsigned char> colors;
    Visualization vis;

    int number_of_frames_hypo_visualized_without_inlier = 8;


//    Vector<Vector<double> > hyposToWrite(HyposMDL.getSize());


    if(Globals::render_bbox3D)
    {

        for(int i = 0; i < HyposMDL.getSize(); i++)
        {

            //***************************************************************************************
            // Render only if the last occurence of an inlier is not far away
            //***************************************************************************************
            Vector<FrameInlier> inlier;
            HyposMDL(i).getIdx(inlier);

//            hyposToWrite(i).setSize(5);

            HyposMDL(i).getTrajPts(vvHypoTrajPts);
            Matrix<double> allP;
            HyposMDL(i).getXProj(allP);
            vX = allP.getRow(allP.y_size()-1);
            vX(2) = vX(1);
            vX(1) = vX(3);
            vX.resize(3);
//            hyposToWrite(i)(0) = HyposMDL(i).getHypoID();
//            hyposToWrite(i)(1) = vX(0);
//            hyposToWrite(i)(2) = vX(1);
//            hyposToWrite(i)(3) = vX(2);
//            hyposToWrite(i)(4) = HyposMDL(i).getHeight();

                        if(frame - inlier(inlier.getSize()-1).getFrame() > number_of_frames_hypo_visualized_without_inlier)
                        {
//                            cout << "stop visualization of a Hypo due to missing inliers !!!" << endl;
                            continue;
                        }

            Vector<double> b(3);
            vvHypoTrajPts.clearContent();
            vvHypoTrajPts.setSize(allP.y_size());
            for(int j = 0; j < vvHypoTrajPts.getSize(); j++)
            {
                b(0) = allP(0,j);
                b(1) = allP(3,j);
                b(2) = allP(1,j);
                vvHypoTrajPts(j) = b;
            }

            HyposMDL(i).getDir(vDir);
//            HyposMDL(i).getIdx(Idx);


            vis.render_hypos(cam, frame, assignedBBoxCol, hypoLastSelForVis, possibleColors, im, HyposMDL(i).getSpeed(), Globals::minvel, Globals::pedSizeWVis,
                             Globals::pedSizeWVis, HyposMDL(i).getHeight(), HyposMDL(i).getHypoID(), vvHypoTrajPts, Globals::WORLD_SCALE, vX, vDir, colors);

        }

        if(Globals::render_bbox2D)
        {
            int nrDet = allDet.numberDetectionsAtFrame(frame);
            Vector<double> bbox;
            for(int i = 0; i < nrDet; i++)
            {
                allDet.getBBox(frame, i, bbox);
                vis.render_bbox_2D(bbox, im, 0, 255, 0, 1);
            }
        }

    }

//    AncillaryMethods::exportBBOX(HyposMDL, cam, frame, *aStream);

    //    sprintf(imageSavePath,  Globals::path_to_results_hypos.c_str(), frame);
    //    AncillaryMethods::write_vec_vec_to_disk(hyposToWrite,imageSavePath);

}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////                                               STEREO                                                    ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Tracker::process_frame(Detections& det, Camera &cam, int frameNr,  Vector< Hypo >& HyposAll)
{


    MDL mdl;
    Vector < FrameInlier > currIdx;
    Vector < FrameInlier > oldIdx;
    Vector < FrameInlier > overlap;
    Vector <int> vRemoveHypos;
    int hypoIdNew;
    Vector<int> HypoIdx;
    double bestFraction = Globals::dSameIdThresh;
    int bestNumMatches = 0;
    int numObsCurr;
    int numObsOld;
    int numOverlap;
    double equalFraction;
    int bTerminated = 0;
    Matrix<double> Q;
    Vector<double> m;
    Vector< Hypo > HyposNew;
    Vector< Hypo > HypoExtended;


    double normfct = 0.3;
    //    double normfct = 1.5*max(0.2,(0.1 + 0.9*(1-(1./Globals::history)*t)));
    //*****************************************************************
    // Define frame range for finding new Hypos
    //*****************************************************************

    int LTPmax = frameNr;
    int LTPmin = max(LTPmax-Globals::history, Globals::nOffset);

    //******************************************************************
    // Extend Trajectories Hypos
    //******************************************************************

    Vector<int> extendUsedDet;

    // this method extends existing hypothesis'
    extend_trajectories(HyposAll,  det, LTPmax, LTPmin, normfct, HypoExtended, extendUsedDet/*, cam*/);
//    if(Globals::verbose){
//        cout << "\33[36;40;1m" <<" Extended " << HypoExtended.getSize()
//             << " trajectories" << "\33[0m" << endl;
//    }
    ROS_DEBUG("\33[36;40;1m Extended %i trajectories\33[0m", HypoExtended.getSize());

    //        extendUsedDet.clearContent();
    //******************************************************************
    // Find new Hypos
    //******************************************************************

    make_new_hypos(LTPmax, LTPmin, det, HyposNew, normfct, extendUsedDet);
//    if(Globals::verbose){
//        cout<< "\33[31;40;1m" << "     Created " << HypoNew.getSize()
//            << " new Trajectories " << "\33[0m"  << endl;
//    }
    ROS_DEBUG("\33[31;40;1m     Created %i new Trajectories \33[0m", HyposNew.getSize());

    HyposAll.clearContent();
//    HyposAll.append(HypoEnded);
    HyposAll.append(HypoExtended);
    HyposAll.append(HyposNew);
    HyposNew.clearContent();
    HypoExtended.clearContent();

    //******************************************************************
    // Prepare Hypos
    //******************************************************************
    prepare_hypos(HyposAll);

    Vector <Hypo> temp;

    for (int i = 0; i < HyposAll.getSize(); i++)
    {
        if (HyposAll(i).getScoreMDL() > Globals::dTheta2)
        {
            temp.pushBack(HyposAll(i));
        }
    }

    HyposAll = temp;
    
    //******************************************************************
    // Build the MDL Matrix
    //******************************************************************

    mdl.build_mdl_matrix(Q, HyposAll, LTPmax, normfct);

    //******************************************************************
    //  Solve MDL Greedy
    //******************************************************************

    //    mdl.solve_mdl_exactly(Q, m, HyposMDL, HypoIdx, HyposAll);

    mdl.solve_mdl_greedy(Q, m, HyposMDL, HypoIdx, HyposAll);
    //******************************************************************
    //  Fix IDs
    //******************************************************************

    for(int i = 0; i < HyposMDL.getSize(); i++)
    {
        // skip terminated hypotheses


        if (HyposMDL(i).isTerminated()) continue;
        bTerminated = 0;
        // if the parent is known from trajectory extension
        if (HyposMDL(i).getParentID() > 0)
        {
            HyposMDL(i).setHypoID(HyposMDL(i).getParentID());
//            if(Globals::verbose){
//                printf("Continuing extended trajectory %d (%f - %f) \n", HyposMDL(i).getHypoID(), (1 - Globals::k2)*HyposMDL(i).getNW(), Globals::k2*HyposMDL(i).getScoreW());
//            }
            ROS_DEBUG("Continuing extended trajectory %d (%f - %f) \n", HyposMDL(i).getHypoID(), (1 - Globals::k2)*HyposMDL(i).getNW(), Globals::k2*HyposMDL(i).getScoreW());
            for (int j = 0; j < hypoStack.getSize(); j++)
            {
                if (hypoStack(j).getHypoID() == HyposMDL(i).getHypoID())
                {
                    hypoStack(j) = HyposMDL(i);
                    break;
                }
            }
        }
        else
        {
            //if the parent is unknown - determine overlap with existing hypothesis
            hypoIdNew = -1;
            bestNumMatches = 0;
            int lengthOldStack = hypoStack.getSize();
            HyposMDL(i).getIdx(currIdx);
            numObsCurr = AncillaryMethods::getSizeIdx(currIdx);

            // check the hypothesis against the stack of previous ones (try to
            // find the trajectory ID in the past)
            for(int j = 0; j < lengthOldStack; j++)
            {
                hypoStack(j).getIdx(oldIdx);
                AncillaryMethods::intersectIdx(currIdx, oldIdx, overlap);
                numObsOld = AncillaryMethods::getSizeIdx(oldIdx);
                numOverlap = AncillaryMethods::getSizeIdx(overlap);

                if(overlap.getSize() > 0)
                {
                    equalFraction = double(numOverlap) / min(double(numObsCurr), double(numObsOld));
                }
                else
                {
                    equalFraction = 0.0;
                }

                if(equalFraction > bestFraction && numOverlap > bestNumMatches)
                {
                    hypoIdNew = hypoStack(j).getHypoID();
                    bestNumMatches = numOverlap;
                }
            }

            if (hypoIdNew > -1)
            {
                //ID was found - set ID of current trajectory, and update the stack

                HyposMDL(i).setHypoID(hypoIdNew);
                if(bTerminated == 0)
                {
                    hypoStack(hypoIdNew) = (HyposMDL(i));
                }else
                {
                    vRemoveHypos.pushBack(i);
                }

//                if(Globals::verbose){
//                    printf("Replacing trajectory %d with new hypos (%f - %f) \n", HyposMDL(i).getHypoID(), (1 - Globals::k2)*HyposMDL(i).getNW(), Globals::k2*HyposMDL(i).getScoreW());
//                }
                ROS_DEBUG("Replacing trajectory %d with new hypos (%f - %f) \n", HyposMDL(i).getHypoID(), (1 - Globals::k2)*HyposMDL(i).getNW(), Globals::k2*HyposMDL(i).getScoreW());
            }
            else
            {
                // ID was not found - create a new ID for the current trajectory
                // and add it to the stack

                lastHypoID +=1;
                HyposMDL(i).setHypoID(lastHypoID);
                hypoStack.pushBack(HyposMDL(i));
//                if(Globals::verbose){
//                    cout << "Creating new Trajectory " << HyposMDL(i).getHypoID() << " (" << (1 - Globals::k2)*HyposMDL(i).getNW() << " - " << Globals::k2*HyposMDL(i).getScoreW() << ") "<<  endl;
//                }
                ROS_DEBUG("Creating new Trajectory %i (%f - %f) ", HyposMDL(i).getHypoID(), (1 - Globals::k2)*HyposMDL(i).getNW(), Globals::k2*HyposMDL(i).getScoreW());
            }
        }
    }

    //******************************************************************
    // Print HypoMDL results
    //******************************************************************

    for(int i = 0; i < HyposMDL.getSize(); i++)
    {
        if(HyposMDL(i).isMoving())
        {
//            if(Globals::verbose){
//                cout << "\33[1;32;40;1m" << "Score of Hypo "<<  HyposMDL(i).getHypoID() << " is = " << HyposMDL(i).getScoreMDL() << " (pedestrian, moving, speed = " << HyposMDL(i).getSpeed() << " )" << "\33[0m" << endl;
//            }
            ROS_DEBUG("\33[1;32;40;1m Score of Hypo %i is = %f (pedestrian, moving, speed = %f )\33[0m", HyposMDL(i).getHypoID(), HyposMDL(i).getScoreMDL(), HyposMDL(i).getSpeed());
        }
        else
        {
//            if(Globals::verbose){
//                cout << "\33[1;32;40;1m" << "Score of Hypo "<<  HyposMDL(i).getHypoID() << " is = " << HyposMDL(i).getScoreMDL() << " (pedestrian, static) " << "\33[0m" << endl;
//            }
            ROS_DEBUG("\33[1;32;40;1m Score of Hypo %i is = %f (pedestrian, static) \33[0m", HyposMDL(i).getHypoID(), HyposMDL(i).getScoreMDL());
        }
    }

    for(int i = 0; i < HypoIdx.getSize(); i++)
    {
        if(!HyposMDL(i).isTerminated())
        {
            HyposAll(HypoIdx(i)).setLastSelected(frameNr);
            HyposAll(HypoIdx(i)).setHypoID(HyposMDL(i).getHypoID());
            HyposAll(HypoIdx(i)).setParentID(HyposMDL(i).getParentID());
        }
    }

    check_termination(cam, HyposAll);

    //******************************************************************
    // Only propagate non-terminated hypotheses to the next frame
    //******************************************************************

    unsigned int nr =  HyposAll.getSize();
    temp.clearContent();
    for(unsigned int i = 0; i < nr; i++)
    {
        if(!HyposAll(i).isTerminated() && (frameNr - HyposAll(i).getLastSelected()) < Globals::coneTimeHorizon)
        {
            temp.pushBack(HyposAll(i));
        }
    }

    HyposAll = temp;
}

void Tracker::prepare_hypos(Vector<Hypo>& vHypos)
{

    int nrHyposOld  = vHypos.getSize();
    int nrHyposOldOut = nrHyposOld;

    double k1 = Globals::k1;
    double k2 = Globals::k2;

    double nw;
    double scoreW;
    double scoreMDL;
    Vector<int> vTrajT;
    Vector<FrameInlier> Idx;

    //**********************************************************************
    // Prepare the hypothesis scores
    //**********************************************************************

    for (int i = 0; i < nrHyposOld; i++)
    {
        if (vHypos(i).getSpeed() > Globals::minvel)
        {
            vHypos(i).setMoving(true);
        }else
        {
            vHypos(i).setMoving(false);
        }
    }

    for(int i = 0; i < nrHyposOld; i++)
    {


        nw = vHypos(i).getNW();
        scoreW = vHypos(i).getScoreW();
        scoreMDL = -k1 + ((1-k2)*nw + k2*scoreW);
        vHypos(i).setScoreMDL(scoreMDL);

        vHypos(i).getTrajT(vTrajT);
        vHypos(i).getIdx(Idx);


        int nrInlier = 0;

        for(int j = 0; j < Idx.getSize(); j++)
        {
            nrInlier += Idx(j).getNumberInlier();
        }

        if(/*vHypos(i).isMoving() &&*/ nrInlier < Globals::threshLengthTraj)
        {
            vHypos(i).setScoreMDL(-1.0);
        }
    }

//    if(Globals::verbose){
//        cout << "Filtering out low-scoring hypothesis..." << endl;
//        cout << "  compacted hypothesis set from " << nrHyposOldOut << " to " << vHypos.getSize() << endl;
//    }
    ROS_DEBUG("Filtering out low-scoring hypothesis...");
    ROS_DEBUG("  compacted hypothesis set from %i to %i", nrHyposOldOut, vHypos.getSize());


    //*************************************************************************
    // Remove duplicates
    //*************************************************************************

    remove_duplicates(vHypos);
}

void Tracker::remove_duplicates(Vector<Hypo>& hypos)
{
    Vector<int> duplicats(hypos.getSize(), -1);
    Vector<FrameInlier> idx1;
    Vector<FrameInlier> idx2;
    Vector<FrameInlier> intersection;

    int nrHyposOld = hypos.getSize();


    //*******************************************************************
    // Find Duplicates
    //*******************************************************************

    for(int i = 0; i < hypos.getSize(); i++)
    {
        if(duplicats(i)==-1)
        {
            for(int j = i+1; j < hypos.getSize(); j++)
            {
                if(duplicats(j)==-1 && hypos(i).isMoving() == hypos(j).isMoving())
                {
                    hypos(i).getIdx(idx1);
                    hypos(j).getIdx(idx2);
                    AncillaryMethods::intersectIdx(idx1, idx2, intersection);

                    if(AncillaryMethods::getSizeIdx(idx1)*0.89 < AncillaryMethods::getSizeIdx(intersection) || AncillaryMethods::getSizeIdx(idx2)*0.89 < AncillaryMethods::getSizeIdx(intersection))
                    {
                        if(hypos(i).getScoreMDL() >= hypos(j).getScoreMDL())
                        {
                            duplicats(j) = 1;
                        }
                        else
                        {
                            duplicats(i) = 1;
                        }
                    }
                }
            }
        }
    }

    //***********************************************************************
    // Remove Duplicates
    //***********************************************************************

    Vector<Hypo> copyH;
    for(int i = 0; i < hypos.getSize(); i++)
    {
        if(duplicats(i)==-1)
        {
            copyH.pushBack(hypos(i));
        }
    }

    hypos = copyH;
//    if(Globals::verbose){
//        cout << "Removing duplicate hypos..." << endl;
//        cout << "  compacted hypothesis set from " << nrHyposOld << " to " << hypos.getSize() << endl;
//    }
    ROS_DEBUG("Removing duplicate hypos...");
    ROS_DEBUG("  compacted hypothesis set from %i to %i", nrHyposOld, hypos.getSize());
}


void getCurrentSmoothDirection(Matrix<double> &pts, int smoothing_window, double &dir, double &vel)
{

    Matrix<double> smoothed = AncillaryMethods::smoothTrajMatrix(pts, smoothing_window);
    smoothed = AncillaryMethods::smoothTrajMatrix(smoothed, smoothing_window);

    Vector<double> xs = smoothed.getRow(1);
    Vector<double> xe = smoothed.getRow(min(smoothed.y_size()-1, smoothing_window));

    xe -= xs;
    xe *= 1.0/(double)(min(smoothed.y_size(), smoothing_window));

    dir = atan2(xe(1),xe(0));
    vel = sqrtf(xe(0)*xe(0)+xe(1)*xe(1));

}

void Tracker::extend_trajectories(Vector<Hypo>& allHypos,  Detections& det, int t, int /*LTPmin*/,
                                  double normfct, Vector<Hypo>& HypoExtended, Vector<int>& extendUsedDet/*,
                                  Camera &*/ /*cam*/)
{


    //*************************************************************************
    // Create trajectory hypotheses by extending the previously existing
    //*************************************************************************

    extendUsedDet.clearContent();
    Vector<int> extendedUseTemp;

    Vector<double> Lmax(3, Globals::pedSizeWCom);
    Lmax(2) = Globals::pedSizeHCom;

    double dLastT;
    int nrFramesWithInlier;
    Vector<double> vMeanPoint3D(3, 0.0);

    Matrix<double> Rot4D;


    Hypo* auxHypo;
    Vector< Hypo > newHypos;

    Matrix<double> mAllXnewUp;
    Vector<FrameInlier> vvIdxUp;
    Vector<double>  vRUp;
    Vector<double>  vVUp;

    Matrix<double> mNewAllX;
    Vector<FrameInlier> vvNewIdx;
    Vector<double>  vNewR;
    Vector<double>  vNewV;

    Matrix<double> mCurrAllX;
    Vector<FrameInlier> vvCurrIdx;
    Vector<double>  vCurrR;
    Vector<double>  vCurrV;

    Volume<double> volHMean;
    int pointPos;
    Vector<int> inlier;
    int n_HypoId;
    Vector<double> pos3DStl;


    int timeHorizon = Globals::coneTimeHorizon;

    //*************************************
    // Kalman
    //*************************************

    Vector<Matrix<double> > stateCovMatsNew;
    Vector<Volume<double> > colHistsNew;

    Vector<Matrix<double> > stateCovMatsOld;
    Vector<Volume<double> > colHistsOld;


    int allhyposSize = allHypos.getSize();
    for (int i = 0; i < allhyposSize; i++)
    {

        auxHypo = &(allHypos(i));

        //**********************************************************
        // if a hypothesis was useless for too long, abandon it
        //***********************************************************

        if(t - auxHypo->getLastSelected() > timeHorizon)
        {
//            if(Globals::verbose){
//                cout << "  DEBUG: Hypothesis " << i << " too old ==> dropped." << endl;
//            }
            ROS_DEBUG("  DEBUG: Hypothesis %i too old ==> dropped.", i);
            continue;
        }

        auxHypo->getIdx(vvCurrIdx);
        auxHypo->getV(vCurrV);
        auxHypo->getR(vCurrR);
        auxHypo->getXProj(mCurrAllX);
        auxHypo->getRot4D(Rot4D);
        n_HypoId = auxHypo->getHypoID();

        nrFramesWithInlier = vvCurrIdx.getSize();
        dLastT = vvCurrIdx(nrFramesWithInlier-1).getFrame();

        vMeanPoint3D.fill(0.0);
        inlier = vvCurrIdx(nrFramesWithInlier-1).getInlier();

        Vector<double> bbox(4,0.0);
        Vector<double>bbox_sum(4,0.0);
        for(int j = 0; j < inlier.getSize(); j++)
        {
            det.getPos3D(dLastT, inlier(j), pos3DStl);
            vMeanPoint3D(0) += pos3DStl(0);
            vMeanPoint3D(1) += pos3DStl(1);
            vMeanPoint3D(2) += pos3DStl(2);
            det.getBBox(dLastT, inlier(j), bbox);
            bbox_sum += bbox;

        }

        vMeanPoint3D *=(1.0/(inlier.getSize()));
        bbox_sum *=(1.0/(inlier.getSize()));

        bbox = bbox_sum;

        double minValue = 100000.0;

        for(int j = 0; j < det.numberDetectionsAtFrame(dLastT); j++)
        {
            det.getPos3D(dLastT, j, pos3DStl);

            pos3DStl -= vMeanPoint3D;

            if (minValue > pos3DStl.norm())
            {
                minValue = pos3DStl.norm();
                pointPos = j;
            }
        }

        Hypo newHypo;

        Matrix<double> mXProj;
        auxHypo->getXProj(mXProj);

        Vector<double> xInit;

        Vector<Vector<double> > vvTrajPTS;
        auxHypo->getTrajPts(vvTrajPTS);

        Vector<double> rotation;
        auxHypo->getR(rotation);


        if(vvTrajPTS.getSize() < 2) continue;


        xInit.setSize(4, 0.0);

        xInit(0) = mXProj(0, mXProj.y_size()-1);
        xInit(1) = mXProj(1, mXProj.y_size()-1);

        double dir, vel;



        getCurrentSmoothDirection(mXProj, 12, dir, vel);


//                        xInit(2) = rotation(rotation.getSize()-1);
//                        xInit(3) = vCurrV(vCurrV.getSize()-1);

        xInit(2) = dir;
        xInit(3) = vel*Globals::frameRate;
//        if(vel < Globals::minvel*3)
//            xInit(3) = 0;

        auxHypo->getColHists(colHistsOld);
        auxHypo->getStateCovMats(stateCovMatsOld);

        EKalman kalman;
        kalman.init(xInit, stateCovMatsOld(stateCovMatsOld.getSize()-1), 1.0/Globals::frameRate);
        kalman.runKalmanUp(det, t-1, t, mAllXnewUp, vvIdxUp, vRUp, vVUp, volHMean,
                           stateCovMatsNew, colHistsOld(colHistsOld.getSize()-1), colHistsNew,
                           mXProj(3, mXProj.y_size()-1), bbox);

        // calcualte new Size for allX, V, R, W ...

        if(vvIdxUp.getSize() > 0)
            extendedUseTemp = vvIdxUp(0).getInlier();

        if(extendedUseTemp.getSize() > 0)
            extendUsedDet.append( extendedUseTemp);



        int newSize = mXProj.y_size() + 1;

        mNewAllX.set_size(4, newSize);
        vNewR.setSize(newSize);
        vNewV.setSize(newSize);

        // get the part of the original trajectory up to, but excluding, the last detection
        // and add the newly appened part


        for(int j = 0; j < mXProj.y_size(); j++)
        {
            vNewR(j) = vCurrR(j);
            vNewV(j) = vCurrV(j);
        }

        for(int j = mXProj.y_size(); j < newSize; j++)
        {
            vNewR(j) = vRUp(j -  (mXProj.y_size()));
            vNewV(j) = vVUp(j -  (mXProj.y_size()));
        }


        for(int j = 0; j < vvCurrIdx.getSize(); j++)
        {
            vvNewIdx.pushBack(vvCurrIdx(j));
        }

        vvNewIdx.append(vvIdxUp);

        stateCovMatsOld.append(stateCovMatsNew);
        colHistsOld.append(colHistsNew);

        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < mXProj.y_size(); k++)
            {
                mNewAllX(j,k) = mCurrAllX(j,k);
            }
        }

        for(int j = 0; j < 4; j++)
        {
            for(int k = mXProj.y_size(); k < newSize ; k++)
            {
                mNewAllX(j,k) = mAllXnewUp(j,k - (mXProj.y_size()));
            }
        }

        // FIXME
        Matrix<double> pp;
        mNewAllX.cutPart(0,1,0,mNewAllX.y_size()-1,pp);
        Matrix<double> smoothed = AncillaryMethods::smoothTrajMatrix(pp, 12);
        smoothed = AncillaryMethods::smoothTrajMatrix(smoothed, 12);
        mNewAllX.insert(pp,0,0);


        // REDUCING TRAJECTORIE LENGTH

        int max_length = Globals::frameRate*0.5;

        if(mNewAllX.y_size()>max_length)
//        if(false)
        {
            Matrix<double> cutMat;
            mNewAllX.cutPart(0,mNewAllX.x_size()-1, mNewAllX.y_size()-max_length, mNewAllX.y_size()-1, cutMat);
            mNewAllX = cutMat;
//            vNewR.swap();
//            vNewR.resize(max_length);
//            vNewR.swap();
            vNewR.resize_from_end(max_length);

//            vNewV.swap();
//            vNewV.resize(max_length);
//            vNewV.swap();
            vNewV.resize_from_end(max_length);

            int first_frame = mNewAllX(2, 0);

            int counter = 0;

            for(int j=vvNewIdx.getSize()-1; j >=0; j--)
            {
                if(vvNewIdx(j).getFrame()>=first_frame){
                    counter += 1;
                    continue;
                }
                else{
                    break;
                }
            }

//            vvNewIdx.swap();
//            vvNewIdx.resize(counter);
//            vvNewIdx.swap();
            vvNewIdx.resize_from_end(counter);
        }


        compute_hypo_entries(mNewAllX, vNewR, vNewV, vvNewIdx, det, newHypo, normfct, t);
        newHypo.setParentID(n_HypoId);
        newHypo.setStateCovMats(stateCovMatsOld);
        newHypo.setColHists(colHistsOld);


        ///////////// copying the old stuff into the new hypo
        for(int j=0; j<auxHypo->getUbdSeqNr().size();j++){
        	newHypo.pushUbdIndex(auxHypo->getUbdIndex().at(j));
        	newHypo.pushUbdSeqNr(auxHypo->getUbdSeqNr().at(j));
        }

        // and extending it with new stuff
        newHypo.pushUbdSeqNr(det.getSeqNr(t, i));
        newHypo.pushUbdIndex(det.getIndex(t, i));
        /////////////////////////////////////////


        if (newHypo.getCategory() != -1)
        {
            newHypos.pushBack(newHypo);
        }
        vvNewIdx.clearContent();
        vvIdxUp.clearContent();
    }

    HypoExtended.clearContent();

    for(int i = 0; i < newHypos.getSize(); i++)
    {
        HypoExtended.pushBack(newHypos(i));
    }
}

void Tracker::make_new_hypos(int endFrame, int tmin, Detections& det, Vector< Hypo >& hyposNew,  double normfct, Vector<int>& extendUsedDet)
{
    Vector<double> xInit;
    Matrix<double> PInit;
    Vector<double> pos3d;

    double dt = 1.0 / Globals::frameRate;


    PInit.set_size(4,4, 0.0);

    PInit(0,0) = 0.4;
    PInit(1,1) = 1.2;
    PInit(2,2) = 0.2;
    PInit(3,3) = 0.2;


    Matrix<double> mAllXnewDown;
    Volume<double> volHMean;
    Vector<FrameInlier> vvIdxDown;
    Vector<double>  vRDown;
    Vector<double>  vVDown;

    Vector<Matrix<double> > stateCovMats;
    Vector<Volume<double> > colHists;

    double v = Globals::dMaxPedVel/3.0;

    double r = M_PI; // Assume as Prior that the persons are front orientated.

    hyposNew.clearContent();

//    Vector<double> bbox;


    int nrOfDet = det.numberDetectionsAtFrame(endFrame);


    for(int j = 0; j < nrOfDet; j++)
    {
        if(extendUsedDet.findV(j) >= 0)
            continue;

//        det.getBBox(endFrame, j, bbox);
        det.getPos3D(endFrame, j, pos3d);

        xInit.setSize(4);
        xInit(0) = pos3d(0);
        xInit(1) = pos3d(2);
        xInit(2) = r;
        xInit(3) = v;

        EKalman kalman;
        kalman.init(xInit, PInit, dt);

        kalman.runKalmanDown(det, endFrame, j, tmin, mAllXnewDown, vvIdxDown, vRDown, vVDown, volHMean, stateCovMats, colHists);

        //*********************************
        // swap data to make it consistent
        //*********************************
        vRDown.swap();
        vVDown.swap();
        vvIdxDown.sortV();
        mAllXnewDown.swap();


        // SMOOTHING
        //
        //        // FIXME

        Matrix<double> pp;
        mAllXnewDown.cutPart(0,1,0,mAllXnewDown.y_size()-1,pp);
        Matrix<double> smoothed = AncillaryMethods::smoothTrajMatrix(pp, 12);
        smoothed = AncillaryMethods::smoothTrajMatrix(smoothed, 12);
        mAllXnewDown.insert(pp,0,0);

        AncillaryMethods::swapVectorMatrix(stateCovMats);
        AncillaryMethods::swapVectorVolume(colHists);

        Hypo hypo;
        hypo.setStateCovMats(stateCovMats);
        hypo.setColHists(colHists);

        hypo.pushUbdSeqNr(det.getSeqNr(endFrame,j));
        hypo.pushUbdIndex(det.getIndex(endFrame, j));


        compute_hypo_entries(mAllXnewDown, vRDown, vVDown, vvIdxDown, det, hypo, normfct, endFrame);

        hypo.setParentID(-1);

        if (hypo.getCategory() != -1)
        {
            hypo.setLastSelected(endFrame);
            hyposNew.pushBack(hypo);
        }
        vvIdxDown.clearContent();
    }
}

void Tracker::compute_hypo_entries(Matrix<double>& allX,  Vector<double>& R, Vector<double>& V, Vector <FrameInlier >& Idx, Detections& det, Hypo& hypo, double normfct, int frame)
{
    // ***********************************************************************
    //   Init
    // ***********************************************************************

    double maxvel = Globals::dMaxPedVel;
    double holePen = Globals::dHolePenalty;
    int maxHoleLen = Globals::maxHoleLen;
    double tau = Globals::dTau;
    int numberInlier = 0;

    for( int i = 0; i < Idx.getSize(); i++)
    {
        numberInlier =  numberInlier + Idx(i).getNumberInlier();
    }

    Vector<double>  Lmax(3);

    Lmax(0) = Globals::pedSizeWCom;
    Lmax(1) = Globals::pedSizeWCom;
    Lmax(2) = Globals::pedSizeHCom;

    int nFrames = 10; // number of Frames for further extropolation of Traj.

    if (numberInlier > 0 && Idx.getSize() > 1)
        //        if (true)
    {

        hypo.setV(V);
        hypo.setR(R);
        hypo.setXProj(allX);
        hypo.setHypoID(-1);

        // ***********************************************************************
        //   Rotation 4D
        // ***********************************************************************

        Matrix<double> rot4D(3, V.getSize(), 0.0);

        for(int i = 0; i < V.getSize(); i++)
        {
            rot4D(0,i) = cos(R(i));
            rot4D(1,i) = sin(R(i));
        }

        hypo.setRot4D(rot4D);

        // ***********************************************************************
        //   Weight, ScoreW, Height
        // ***********************************************************************
        double fct;
        double sumW = 0.0;
        double heightValue = 0.0;
        int currentFrame;
        Vector<int> inlier;
        Vector<double> weights;
        double sumFct = 0;

        for( int i = 0; i < Idx.getSize(); i++)
        {
            currentFrame = Idx(i).getFrame();

            inlier = Idx(i).getInlier();
            weights = Idx(i).getWeight();
            for( int j = 0; j < inlier.getSize(); j++)
            {
                fct = exp(-(frame-currentFrame)/tau);
                weights(j) =  weights(j)*fct;
                sumW += weights(j);
                heightValue += det.getHeight(currentFrame, inlier(j))*fct;
                sumFct += fct;

                hypo.setCategory(det.getCategory(currentFrame, inlier(j)));

            }

            Idx(i).clearWeights();
            Idx(i).setAllWeightsCoincident(weights);
        }

        hypo.setIdx(Idx);
        hypo.setScoreW(sumW);
        hypo.setHeight(heightValue/sumFct);

        // ***********************************************************************
        // Holes // Assume that Idx(frames) are sorted "ascend"
        // ***********************************************************************

        int maxHole = 0;
        int aux;
        int totalHoles = 0;
        for( int i = 1; i < Idx.getSize(); i++)
        {
            //            cout <<" IDX " <<Idx(i).getFrame() << endl;
            aux = Idx(i).getFrame() - Idx(i-1).getFrame();
            totalHoles += aux - 1;
            if(aux > maxHole) maxHole = aux;
        }

        totalHoles += (frame - Idx(Idx.getSize()-1).getFrame());
        maxHole = max(maxHole, frame - Idx(Idx.getSize()-1).getFrame());

        double nw = numberInlier*normfct;
        nw = nw - (holePen*totalHoles*normfct);
        hypo.setNW(nw);


        // ***********************************************************************
        // Hypo Start and Hypo End. // Assume that Idx(frames) are sorted "ascend"
        // ***********************************************************************

        Vector<double>  point(3, 0.0);
        Vector<double>  point4D(4,0.0);
        Vector<double>  oldX;


        Vector <double> pos3DStl;

        //        cout << maxHole << endl;
        if(maxHole < maxHoleLen)
            //        if(true)
        {
            int nrFrWithInl = Idx.getSize();
            inlier = Idx(0).getInlier();

            for( int i = 0; i < inlier.getSize(); i++)
            {
                det.getPos3D(Idx(0).getFrame(), inlier(i), pos3DStl);
                point(0) += pos3DStl(0);
                point(1) += pos3DStl(1);
                point(2) += pos3DStl(2);
            }

            //Extend the point by the fourth coordinate - the frame number
            point *=(1.0/double(inlier.getSize()));
            oldX = point;
            point4D(0) = point(0);
            point4D(1) = point(1);
            point4D(2) = point(2);
            point4D(3) = Idx(0).getFrame();

            hypo.setStart(point4D);
            point.fill(0.0);

            inlier = Idx(nrFrWithInl-1).getInlier();

            for( int i = 0; i < inlier.getSize(); i++)
            {
                det.getPos3D(Idx(nrFrWithInl-1).getFrame(), inlier(i), pos3DStl);
                point(0) += pos3DStl(0);
                point(1) += pos3DStl(1);
                point(2) += pos3DStl(2);
            }

            point *=(1.0/double(inlier.getSize()));

            //Extend the point by the fourth coordinate - the frame number
            point4D(0) = point(0);
            point4D(1) = point(1);
            point4D(2) = point(2);
            point4D(3) = Idx(nrFrWithInl-1).getFrame();

            hypo.setEnd(point4D);
//            hypo.setX4D(point4D);
//            hypo.setX(point);

            if(Idx(0).getFrame() <= Idx(nrFrWithInl-1).getFrame()) // Changed to <= 22.10.09
            {

                Vector<double>  first;
                Vector<double>  moved;
                Vector<double>  lengthL(2);
                Vector<double>  main4D;
                Vector<double>  up4D(3, 0.0);
                up4D(2) = 1.0;
                Vector<double>  ort4D(3);
                Vector<double>  main3D(3, 0.0);

                // ***********************************************************************
                // First get last and first inlier position / Moved ?
                // ***********************************************************************

                int firstInlPos = -1;
                int lastInlPos = -1;

                int i = 0;

                while(Idx(0).getFrame() != allX(2,i)) i++;
                assert(i < allX.y_size());
                firstInlPos = i;

                i = allX.y_size()-1;

                while(Idx(nrFrWithInl-1).getFrame() != allX(2,i)) i--;
                assert(i >= 0);
                lastInlPos = i;

                Vector<double> assistant(4, 0.0);
                int c = 0;

                for(int j = 1; j < 10; j++)
                {
                    if(allX.y_size()-(j+1)<0)
                        break;

                    allX.getRow((max(0, allX.y_size()-(j+1))), first);
                    allX.getRow((max(0, allX.y_size()-j)), moved);
                    assistant += moved - first;
                    c++;
                }

                if(c > 0)
                {
                    assistant *= (1.0)/double(c);
                }

                assistant.resize(3);
                moved = assistant;

                lengthL(0) = moved(0);
                lengthL(1) = moved(1);

                if (lengthL.norm() != 0)
                {

                    // recover the motion speed
                    hypo.setSpeed((lengthL.norm()/moved(2)));
                    hypo.setMoving(true);

                    main4D = moved;
                    main4D(2) = 0;
                    main4D *=(1.0/main4D.norm());

                    if(main4D.norm() != 0)
                    {
                        // ***********************************************************************
                        // Prepare the spacetime object trajectory projected into 3D
                        // ***********************************************************************

                        ort4D = cross(main4D, up4D); // cross product
                        main3D(0) = main4D(0);
                        main3D(2) = main4D(1);

                        hypo.setDir(main3D);
//                        hypo.setOri4D(main4D);
//                        hypo.setSize(Lmax);

                        Vector<double>  xFirst(3);
                        Vector<double>  xLast(3);
                        Matrix<double> startRect(3, 4);
                        Matrix<double> endRect(3, 4);
//                        Matrix<double> points(3,4, 0.0);

                        allX.getRow(firstInlPos, xFirst);
                        allX.getRow(lastInlPos, xLast);

                        xFirst.resize(3);
                        xLast.resize(3);


                        AncillaryMethods::compute_rectangle(main4D, ort4D, Lmax, xFirst, startRect);
                        hypo.setStartRect(startRect);

                        AncillaryMethods::compute_rectangle(main4D, ort4D, Lmax, xLast, endRect);

                        hypo.setEndRect(endRect);

//                        points(0,0) = endRect(0,0);
//                        points(2,0) = endRect(1,0);
//                        points(0,1) = endRect(0,1);
//                        points(2,1) = endRect(1,1);
//                        points(0,2) = endRect(0,2);
//                        points(2,2) = endRect(1,2);
//                        points(0,3) = endRect(0,3);
//                        points(2,3) = endRect(1,3);

//                        hypo.setPoints(points);

                        // ***********************************************************************
                        // Prepare BBOX for each trajectory point.
                        // ***********************************************************************

                        //******** Necessary for linear interpol. of missing trajrect. **********
                        Matrix<double> mOldRect(3,4);
                        Matrix<double> mD;
                        Matrix<double> mRj;
                        Vector<double> vXj;
                        Vector<double> copyX3D;
                        // ***********************************************************************

                        int startFrame = Idx(0).getFrame();
                        int endFrame = Idx(nrFrWithInl-1).getFrame();

                        int nMissing = 0;
                        int LoopCounter = 0;
                        int countAllMissing = 0;

                        int nrFrBetweenStartEnd = endFrame - startFrame + 1; // include the start as well the end frame

                        Vector<double> x3D(3);
                        Vector<Matrix <double> > TrajRect;
                        Vector<Vector < double > > TrajPts;
                        Vector <int> TrajT;
                        Matrix<double> rect(3,4);

                        mOldRect = startRect;
                        int cp_nrFrBetweenStartEnd = nrFrBetweenStartEnd;

                        for(int i = 0; i < cp_nrFrBetweenStartEnd; i++)
                        {

                            allX.getRow(firstInlPos + i, point);

                            if (point(2) == startFrame + i + LoopCounter)
                            {

                                x3D(0) = point(0);
                                x3D(1) = point(3);
                                x3D(2) = point(1);

                                point.resize(3);

                                AncillaryMethods::compute_rectangle(main4D, ort4D, Lmax, point, rect);

                                //---------------------------------------------------------
                                // Fill in missing trajectory bboxes by linear interpolation
                                //---------------------------------------------------------

                                for(int j = 0; j < nMissing; j++)
                                {

                                    mD = rect-mOldRect;

                                    mD *= (1.0/(nMissing + 1));
                                    mD *=(j+1);
                                    mRj = mOldRect+mD;

                                    copyX3D = x3D - oldX;
                                    copyX3D *=(j+1);
                                    copyX3D *= (1.0/(nMissing + 1));
                                    vXj = oldX + copyX3D;

                                    TrajRect.pushBack(mRj);
                                    TrajPts.pushBack(vXj);
                                    TrajT.pushBack(startFrame + i + countAllMissing);

                                    countAllMissing +=1;
                                }

                                TrajRect.pushBack(rect);

                                TrajPts.pushBack(x3D);
                                TrajT.pushBack(startFrame + i + countAllMissing);

                                oldX = x3D;
                                mOldRect = rect;
                                nMissing = 0;
                            }
                            else
                            {
                                LoopCounter += 1;
                                nMissing += 1;
                                i = i - 1;
                                cp_nrFrBetweenStartEnd = cp_nrFrBetweenStartEnd - 1;
                            }
                        }

                        //************************************************************************
                        // Predict up to nFrames extra frames
                        //************************************************************************

                        int posT = 0;
                        int len = min(frame-max(maxHoleLen, 10), endFrame - 1);

                        if(len >= 0 && len >= startFrame)
                        {
                            if(endFrame - len >= 0)
                            {
                                posT = len - startFrame;
                            }
                            else
                            {
                                posT = nrFrBetweenStartEnd - 1;
                            }
                        }

                        // Init for the prediction procedure

                        Vector<double> x1;
                        Vector<double> x2;

                        Matrix<double> rect1;
                        Matrix<double> rect2;
                        int tlen;
                        Vector<double> v;
                        Matrix<double> vrect;
                        double vnorm;

                        // I have to do this distinction since at the begining the
                        // trajectories contain only 2 Points.

                        x1 = TrajPts(posT);
                        x2 = TrajPts(TrajPts.getSize() - 1);
                        rect1 = TrajRect(posT);
                        rect2 = TrajRect(TrajRect.getSize() - 1);
                        tlen = TrajPts.getSize() - 1 - posT;
                        v = x2;
                        v -= x1;
                        v *=(1.0/tlen);
                        vrect = rect2;
                        vrect -= rect1;
                        vrect *=(1.0 / tlen);

                        vnorm = v.norm();

                        if (vnorm > maxvel)
                        {
                            v *= (maxvel/vnorm);
                            vrect *=(maxvel/vnorm);
                            //cout << "WARNING(compute_hypo): Extrapolation exceeds maxvel" << endl;
                        }

                        for(int ff = 0; ff < vrect.y_size(); ff++)
                        {
                            vrect(2, ff) = 1.0;
                        }
                        //                        vrect.fillColumn(1.0, 2);

                        Matrix<double> copyVrect;
                        Vector<double> copyV;

                        // Change ... 22.02.2010 delete +1 by i <
                        for (int i = endFrame; i < min(endFrame + nFrames, frame); i++)
                        {
                            mRj = rect2;
                            copyVrect = vrect;

                            copyVrect *=(i - endFrame);
                            mRj += (copyVrect);

                            copyV = v;
                            copyV *= (i - endFrame);

                            vXj = x2;
                            vXj +=copyV;

                            TrajRect.pushBack(mRj);
                            TrajPts.pushBack(vXj);
                            TrajT.pushBack(startFrame + nrFrBetweenStartEnd + (i - endFrame));

                        }


                        hypo.setTrajRect(TrajRect);
                        hypo.setTrajPts(TrajPts);
                        hypo.setTrajT(TrajT);

                        // ***********************************************************************
                        // Precompute a spacetime bbox for the hypothesis.
                        // ***********************************************************************

                        hypo.getStartRect(startRect);
                        hypo.getEndRect(endRect);

                        Vector<double> start0;
                        Vector<double> start1;
                        Vector<double> end0;
                        Vector<double> end1;

                        start0 = startRect.getColumn(0);
                        start1 = startRect.getColumn(1);
                        end0 = endRect.getColumn(0);
                        end1 = endRect.getColumn(1);

                        double xmin = min(start0.minim().first, end0.minim().first);
                        double xmax = max(start0.maxim().first, end0.maxim().first);
                        double ymin = min(start1.minim().first, end1.minim().first);
                        double ymax = max(start1.maxim().first, end1.maxim().first);

                        hypo.getStart(start0);
                        hypo.getEnd(end0);
                        Matrix<double> bbox(3,2);
                        bbox(0,0) = xmin;
                        bbox(1,0) = ymin;
                        bbox(2,0) = start0(3);
                        bbox(0,1) = xmax;
                        bbox(1,1) = ymax;
                        bbox(2,1) = end0(3);

                        hypo.setBBox4D(bbox);

                    }
                    else
                    {
                        hypo.setCategory(-1);
//                        if(Globals::verbose){
//                            cerr << "hypo has no main direction => reject!" << endl;
//                        }
                        ROS_DEBUG("hypo has no main direction => reject!");
                    }
                }
                else
                {
                    hypo.setSpeed(0);
                    hypo.setCategory(-1);
//                    if(Globals::verbose){
//                        cerr << "Hypo " << hypo.getHypoID()  << " is not moving => reject!" << endl;
//                    }
                    ROS_DEBUG("Hypo %i is not moving => reject!", hypo.getHypoID());
                }
            }
            else
            {
                hypo.setCategory(-1);
//                if(Globals::verbose){
//                    cerr << " Hypo contains only single frame => reject! " << endl;
//                }
                ROS_DEBUG(" Hypo contains only single frame => reject! ");
            }
        }
        else
        {
            hypo.setCategory(-1);
//            if(Globals::verbose){
//                cerr << "Hypo " << hypo.getHypoID() << " had large holes : MaxHoleLength - " << maxHole << " => reject" << endl;
//            }
            ROS_DEBUG("Hypo %i had large holes : MaxHoleLength - %i => reject", hypo.getHypoID(), maxHole);
        }
    }else
    {
        hypo.setCategory(-1);
//        if(Globals::verbose){
//            cout << "Size of Idx is 1, so no hypo can be computed" << endl;
//        }
        ROS_DEBUG("Size of Idx is 1, so no hypo can be computed");
    }
}
