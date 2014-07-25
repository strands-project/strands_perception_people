/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto                                 *
 *   nbellotto@lincoln.ac.uk                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef SIMPLE_TRACKING_H
#define SIMPLE_TRACKING_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <bayestracking/multitracker.h>
#include <bayestracking/models.h>
#include <bayestracking/ukfilter.h>
#include <cstdio>
#include <boost/thread.hpp>

using namespace std;
using namespace MTRK;
using namespace Models;


typedef UKFilter Filter;

// rule to detect lost track
template<class FilterType>
bool MTRK::isLost(const FilterType* filter) {
    // track lost if var(x)+var(y) > 1
    if (filter->X(0,0) + filter->X(2,2) > sqr(1.0))
        return true;
    return false;
}

// rule to create new track
template<class FilterType>
bool MTRK::initialize(FilterType* &filter, sequence_t& obsvSeq) {
    assert(obsvSeq.size());

    double dt = obsvSeq.back().time - obsvSeq.front().time;
    assert(dt); // dt must not be null
    FM::Vec v((obsvSeq.back().vec - obsvSeq.front().vec) / dt);

    FM::Vec x(4);
    FM::SymMatrix X(4,4);

    x[0] = obsvSeq.back().vec[0];
    x[1] = v[0];
    x[2] = obsvSeq.back().vec[1];
    x[3] = v[1];
    X.clear();
    X(0,0) = sqr(0.5);
    X(1,1) = sqr(1.5);
    X(2,2) = sqr(0.5);
    X(3,3) = sqr(1.5);

    filter = new FilterType(4);
    filter->init(x, X);

    return true;
}

class SimpleTracking
{
public:
    SimpleTracking() :
        obsvTime(-1),
        alg(NNJPDA) {
        time = getTime();

        cvm = new CVModel(5.0, 5.0);
        ctm = new CartesianModel(0.5, 0.5);
        observation = new FM::Vec(2);

//        tracking_thread = new boost::thread(boost::bind(&SimpleTracking::track, this));
    }

    std::vector<geometry_msgs::Point> track(std::vector<geometry_msgs::Point> obsv) {
       std::vector<geometry_msgs::Point> result;
//        while(ros::ok()){
            ROS_INFO("Tracking");
            dt = getTime() - time;
            time += dt;

            // prediction
            cvm->update(dt);
            mtrk.predict<CVModel>(*cvm);

            // add last observation/s to tracker
            std::vector<geometry_msgs::Point>::iterator li, liEnd = obsv.end();
            for (li = obsv.begin(); li != liEnd; li++) {
                (*observation)[0] = li->x;
                (*observation)[1] = li->y;
                mtrk.addObservation(*observation, obsvTime);
            }

            // process observations (if available) and update tracks
            mtrk.process(*ctm, alg);

            // print
            int n = mtrk.size();

            for (int i = 0; i < n; i++) {
                ROS_INFO("trk_%ld", mtrk[i].id);
                ROS_INFO("Position: (%f, %f), Orientation: %f, Std Deviation: %f",
                         mtrk[i].filter->x[0], mtrk[i].filter->x[2], //x, y
                         atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]), //orientation
                         sqrt(mtrk[i].filter->X(0,0)), sqrt(mtrk[i].filter->X(2,2))//std dev
                         );
                geometry_msgs::Point point;
                point.x = mtrk[i].filter->x[0];
                point.y = mtrk[i].filter->x[2];
                result.push_back(point);
            }
//        }
        return result;
    }

private:
    MultiTracker<Filter, 4> mtrk;    // state [x, v_x, y, v_y]
    CVModel *cvm;                    // CV model with sigma_x = sigma_y = 5.0
    CartesianModel *ctm;             // Cartesian observation model
    FM::Vec *observation;            // observation [x, y]
//    boost::thread *tracking_thread;
    double dt, time, obsvTime;
    const association_t alg;

    double getTime() {
        return ros::Time::now().toSec();
    }

//    void addObservation(std::vector<geometry_msgs::Point> obsv) {
//        // add last observation/s to tracker
//        std::vector<geometry_msgs::Point>::iterator li, liEnd = obsv.end();
//        for (li = obsv.begin(); li != liEnd; li++) {
//            (*observation)[0] = li->x;
//            (*observation)[1] = li->y;
//            mtrk.addObservation(*observation, obsvTime);
//        }
//    }

};
#endif //SIMPLE_TRACKING_H
