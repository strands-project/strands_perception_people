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
#include <geometry_msgs/Pose.h>
#include <bayes_tracking/multitracker.h>
#include <bayes_tracking/models.h>
#include <bayes_tracking/ekfilter.h>
#include <cstdio>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/optional.hpp>
#include <math.h>

using namespace std;
using namespace MTRK;
using namespace Models;


typedef EKFilter Filter;

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
    X(0,0) = sqr(0.2);
    X(1,1) = sqr(1.0);
    X(2,2) = sqr(0.2);
    X(3,3) = sqr(1.0);

    filter = new FilterType(4);
    filter->init(x, X);

    return true;
}

class SimpleTracking
{
public:
    SimpleTracking() {
        time = getTime();
        observation = new FM::Vec(2);
    }

    void addDetectorModel(std::string name, association_t alg, double vel_noise_x, double vel_noise_y, double pos_noise_x, double pos_noise_y) {
        ROS_INFO("Adding detector model for: %s.", name.c_str());
        detector_model det;
        det.alg = alg;
        det.cvm = new CVModel(vel_noise_x, vel_noise_y);
        det.ctm = new CartesianModel(pos_noise_x, pos_noise_y);
        detectors[name] = det;
    }

    std::map<long, std::vector<geometry_msgs::Pose> > track(double* track_time = NULL) {
        boost::mutex::scoped_lock lock(mutex);
        std::map<long, std::vector<geometry_msgs::Pose> > result;
        dt = getTime() - time;
        time += dt;
        if(track_time) *track_time = time;

        for(std::map<std::string, detector_model>::const_iterator it = detectors.begin();
            it != detectors.end();
            ++it) {
            // prediction
            it->second.cvm->update(dt);
            mtrk.predict<CVModel>(*(it->second.cvm));

            // process observations (if available) and update tracks
            mtrk.process(*(it->second.ctm), it->second.alg);
        }

        for (int i = 0; i < mtrk.size(); i++) {
            double theta = atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]);
            ROS_DEBUG("trk_%ld: Position: (%f, %f), Orientation: %f, Std Deviation: %f, %f",
                    mtrk[i].id,
                    mtrk[i].filter->x[0], mtrk[i].filter->x[2], //x, y
                    theta, //orientation
                    sqrt(mtrk[i].filter->X(0,0)), sqrt(mtrk[i].filter->X(2,2))//std dev
                    );
            geometry_msgs::Pose pose, vel;
            pose.position.x = mtrk[i].filter->x[0];
            pose.position.y = mtrk[i].filter->x[2];
            pose.orientation.z = sin(theta/2);
            pose.orientation.w = cos(theta/2);
            result[mtrk[i].id].push_back(pose);

            vel.position.x = mtrk[i].filter->x[1];
            vel.position.y = mtrk[i].filter->x[3];
            result[mtrk[i].id].push_back(vel);
        }
        return result;
    }

    void addObservation(std::string detector_name, std::vector<geometry_msgs::Point> obsv, double obsv_time) {
        boost::mutex::scoped_lock lock(mutex);
        ROS_DEBUG("Adding new observations for detector: %s", detector_name.c_str());
        // add last observation/s to tracker
        detector_model det;
        try {
            det = detectors.at(detector_name);
        } catch (std::out_of_range &exc) {
            ROS_ERROR("Detector %s was not registered!", detector_name.c_str());
            return;
        }

        dt = getTime() - time;
        time += dt;

        // prediction
        det.cvm->update(dt);
        mtrk.predict<CVModel>(*(det.cvm));

        mtrk.process(*(det.ctm), det.alg);

        std::vector<geometry_msgs::Point>::iterator li, liEnd = obsv.end();
        for (li = obsv.begin(); li != liEnd; li++) {
            (*observation)[0] = li->x;
            (*observation)[1] = li->y;
            mtrk.addObservation(*observation, obsv_time);
        }
    }

private:
    MultiTracker<Filter, 4> mtrk;    // state [x, v_x, y, v_y]
    FM::Vec *observation;            // observation [x, y]
    double dt, time;
    boost::mutex mutex;

    struct detector_model {
        CVModel *cvm;               // CV model
        CartesianModel *ctm;        // Cartesian observation model
        association_t alg;          // Data association algorithm
    };
    std::map<std::string, detector_model> detectors;

    double getTime() {
        return ros::Time::now().toSec();
    }
};
#endif //SIMPLE_TRACKING_H
