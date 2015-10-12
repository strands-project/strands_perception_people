#include "mean_shift_utilities.h"
#include <math.h>
#include <iostream>
#include <omp.h>

using namespace std;

MEAN_SHIFT ::MEAN_SHIFT(int total_joints, unsigned samples_per_joint )
{
   // this->totalpoints = total_points;
    this->totaljoints = total_joints;
    this->total_samples_per_joint = samples_per_joint;
    //this->mode_counter = 0;
    //this->nns_to_use = this->total_samples_per_joint;
    this->mode_counter = 0;


}

//function to find peak given  a starting seed point
void MEAN_SHIFT ::findpeaks(float *peak, float & peakscore, \
                            unsigned int *indices, \
                            unsigned int *clustervotes, \
                            Joint *points, \
                            unsigned index, \
                            float bandwidth, long start_position)
{

    double stop_threshold =  bandwidth/1000;
    double bandwidth_square = bandwidth * bandwidth;
    double oldmean[3];
    double newmean[3] ;
    double average_weight = 0;
    //std ::vector<double > tmp;
    //std :: vector<unsigned int> visited(this->nns_to_use);
    double tmp[3];
    unsigned int visited[this->nns_to_use];
    double  euclid_dist;
    double  Weight;


    //doing initializations
    oldmean[0] = 9999;
    oldmean[1] = 9999;
    oldmean[2] = 9999;

    newmean[0] = points[start_position + index].joint_position.x;
    newmean[1] = points[start_position + index].joint_position.y;
    newmean[2] = points[start_position + index].joint_position.z;

    //newmean[0] = index[0];
    //newmean[1] = index[1];
    //newmean[2] = index[2];



    //tmp.push_back(0);
    //tmp.push_back(0);
    //tmp.push_back(0);
    tmp[0] = tmp[1] = tmp[2] = 0;


    peakscore = 0;

    for (int i = 0 ;i < this->nns_to_use ; i ++ )
             visited[i] = 0;

    double threshold = (oldmean[0] - newmean[0])*(oldmean[0] - newmean[0]) + (oldmean[1] - newmean[1])*(oldmean[1] - newmean[1]) + (oldmean[2] - newmean[2])*(oldmean[2] - newmean[2]);
    threshold = sqrt(threshold);
    unsigned pointcounter;



    while (threshold > stop_threshold)
    {
        tmp[0] = tmp[1] = tmp[2] = 0;
        average_weight = 0;
        pointcounter = start_position;
        for (int i = 0; i < this->total_samples_per_joint; i++)
        {

            euclid_dist = (newmean[0] - points[pointcounter].joint_position.x)*(newmean[0] - points[pointcounter].joint_position.x) + (newmean[1] - points[pointcounter].joint_position.y)*(newmean[1] - points[pointcounter].joint_position.y) + (newmean[2] - points[pointcounter].joint_position.z)*(newmean[2] - points[pointcounter].joint_position.z);
            Weight = exp(-euclid_dist/bandwidth_square);
            if (euclid_dist < bandwidth_square)
                {
                    visited[i] = visited[i] + 1;
                    clustervotes[i] = clustervotes[i] + 1;
                    average_weight = average_weight +  Weight;
                    tmp[0] = points[pointcounter].joint_position.x * Weight + tmp[0];
                    tmp[1] = points[pointcounter].joint_position.y * Weight + tmp[1];
                    tmp[2] = points[pointcounter].joint_position.z * Weight + tmp[2];
                   // indices[i] = i;

                }

           pointcounter++;
        }
        //updatting the means
        oldmean[0] = newmean[0];
        oldmean[1] = newmean[1];
        oldmean[2] = newmean[2];

        newmean[0] = tmp[0]/average_weight ;
        newmean[1] = tmp[1]/average_weight ;
        newmean[2] = tmp[2]/average_weight ;

        threshold = (oldmean[0] - newmean[0])*(oldmean[0] - newmean[0]) + (oldmean[1] - newmean[1])*(oldmean[1] - newmean[1]) + (oldmean[2] - newmean[2])*(oldmean[2] - newmean[2]);
        threshold = sqrt(threshold);



    }
    pointcounter = start_position;
    for (int i = 0 ; i < this->nns_to_use ; i++)
    {
        if (visited[i] > 0){
             peakscore = peakscore + points[pointcounter].joint_positions_confidence;
             indices[i] = 1;
           //  std :: cout << "\n" <<indices[i]+1;
        }
    }
    //std :: cout << "\n";
    peak[0] = newmean[0];
    peak[1] = newmean[1];
    peak[2] = newmean[2];
}

void MEAN_SHIFT :: reshape(std::vector<float> &points, \
                 std::vector<std::vector<float> > &Points)
{
    unsigned N = points.size();
    unsigned counter = 0;
    for (int i = 0 ; i < N ; i = i+3)
    {
        //std :: vector<float> tmp;
        Points[counter][0] = points[i];
        Points[counter][1] = points[i + 1];
        Points[counter][2] = points[i + 2];
        counter++;
        //Points.push_back(tmp);
    }
}

//#######################################################################################################
/////////////////////////////function to perform meanshift clustering\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
/// OUTPUTS : joint_postions , joint_positions_confidence
/// INPUTS : points , weights
//#######################################################################################################

void MEAN_SHIFT ::  meanshift(Joint *input_joint_proposals, \
                              float bandwidth, Joint *final_joint_proposals, int index)
{

    int i = 0;
    unsigned int indices[this->total_samples_per_joint];
    unsigned int thisclustervotes[this->total_samples_per_joint];
    unsigned int alreadyassigned[this->total_samples_per_joint];
    Joint tmpjoints[this->total_samples_per_joint];
    int numclusters = 0;
    float Peak[3];
    float PeakScore;
    float dist,min;
    unsigned min_index;

    Peak[0] = Peak[1] = Peak[2];

    for(int j = 0; j < this->total_samples_per_joint ; j++)
    {
        indices[j] = 0;
        thisclustervotes[j] = 0;
        alreadyassigned[j] = 0;
    }
    long start_position = 0;
    //reshape(input_joint_proposals,NewPoints);
    long tmp_peak_counter = 0;
    //this->nns_to_use = this->total_samples_per_joint;
    while ( i < this->total_samples_per_joint)
    {
        if (alreadyassigned[i] == 0)
        {

            findpeaks(Peak,PeakScore,alreadyassigned,thisclustervotes,input_joint_proposals,i,bandwidth,start_position);
            //for (int j = i; j < this->total_samples_per_joint; j ++)
           // {
           //     if (indices[j] == 1)
           //         alreadyassigned[j] = 1;
           // }

            if (i == 0)
            {
                numclusters = numclusters + 1;
                //tmppeaks[0][0] = Peak[0];
                //tmppeaks[0][1] = Peak[1];
                //tmppeaks[0][2] = Peak[2];
                //tmppeakscore[0] = PeakScore;
                tmpjoints[0].joint_position.x = Peak[0];
                tmpjoints[0].joint_position.y = Peak[1];
                tmpjoints[0].joint_position.z = Peak[2];
                tmpjoints[0].joint_positions_confidence = PeakScore;
                //tmpJointId[0] = index;
                tmp_peak_counter++;
              //  this->nns_to_use = this->total_samples_per_joint/4;
            }
            else
                {
                    min = 9999;
                    for (int j = 0 ; j < numclusters ; j++)
                        {

                            //dist = (tmppeaks[j][0] - Peak[0])*(tmppeaks[j][0] - Peak[0]) + (tmppeaks[j][1] - Peak[1])*(tmppeaks[j][1] - Peak[1]) + (tmppeaks[j][2] - Peak[2])*(tmppeaks[j][2] - Peak[2]);
                            dist = (tmpjoints[j].joint_position.x - Peak[0])*(tmpjoints[j].joint_position.x - Peak[0]) + (tmpjoints[j].joint_position.y - Peak[1])*(tmpjoints[j].joint_position.y - Peak[1]) + (tmpjoints[j].joint_position.z - Peak[2])*(tmpjoints[j].joint_position.z - Peak[2]);
                            dist = sqrt(dist);
                            if (dist < min)
                                {
                                    min = dist;
                                    min_index = j;
                                }
                        }
                     if (min <= (bandwidth/2))
                        {
                            //tmppeaks[min_index][0] =  (tmppeaks[min_index][0] + Peak[0])/2;
                            //tmppeaks[min_index][1] =  (tmppeaks[min_index][1] + Peak[1])/2;
                            //tmppeaks[min_index][2] =  (tmppeaks[min_index][2] + Peak[2])/2;
                            //tmppeakscore[min_index] = tmppeakscore[min_index] + PeakScore;
                         tmpjoints[min_index].joint_position.x =  (tmpjoints[min_index].joint_position.x + Peak[0])/2;
                         tmpjoints[min_index].joint_position.y =  (tmpjoints[min_index].joint_position.y + Peak[1])/2;
                         tmpjoints[min_index].joint_position.z =  (tmpjoints[min_index].joint_position.z + Peak[2])/2;
                         tmpjoints[min_index].joint_positions_confidence = tmpjoints[min_index].joint_positions_confidence + PeakScore;

                        }
                    else
                        {
                            numclusters = numclusters + 1;
                            //tmppeaks[tmp_peak_counter][0] = Peak[0];
                            //tmppeaks[tmp_peak_counter][1] = Peak[1];
                            //tmppeaks[tmp_peak_counter][2] = Peak[2];
                            //tmppeakscore[tmp_peak_counter] = PeakScore;
                            tmpjoints[tmp_peak_counter].joint_position.x = Peak[0];
                            tmpjoints[tmp_peak_counter].joint_position.y = Peak[1];
                            tmpjoints[tmp_peak_counter].joint_position.z = Peak[2];
                            tmpjoints[tmp_peak_counter].joint_positions_confidence = PeakScore;
                            //tmpJointId[tmp_peak_counter] = index;
                            tmp_peak_counter++;
                        }

                }
        }
        i = i + 1;

    }

    for (int p = 0; p < tmp_peak_counter ; p++ )
    {
        final_joint_proposals[p].joint_position.x = tmpjoints[p].joint_position.x;
        final_joint_proposals[p].joint_position.y = tmpjoints[p].joint_position.y;
        final_joint_proposals[p].joint_position.z = tmpjoints[p].joint_position.z;
        final_joint_proposals[p].joint_positions_confidence = tmpjoints[p].joint_positions_confidence;
        this->mode_counter++;
    }


}

//##########################################################################################
///////////////////////function to apply mean_shift clustering\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
/// OUTPUTS : joint_positions , joint_positions_confidence
/// INPUTS :  points , weights, bandwidth , JointId
//##########################################################################################

void MEAN_SHIFT ::  apply_meanshift(Joint *final_joint_proposals, \
                                    Joint *input_joint_proposals, \
                                    float bandwidth, unsigned &size)
{


    //this->orientation_increment = orientation_index * this->increment * 9;
    //omp_set_num_threads(8);
    //#pragma omp parallel for
    for(int i = 0 ; i < this->totaljoints ;i++)
    {
        meanshift(input_joint_proposals,bandwidth,final_joint_proposals,i+1);

    }

     size = this->mode_counter;

}
