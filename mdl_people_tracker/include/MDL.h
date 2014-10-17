/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _DENNIS_MDL_H
#define	_DENNIS_MDL_H


#include <algorithm>
#include <map>
#include <omp.h>
#include "Matrix.h"
#include "Hypo.h"
#include "Vector.h"
#include "Math.h"

#include <ros/ros.h>


class MDL
{
    public:
        double solve_mdl_greedy(Matrix<double>& Q, Vector<double>& m, Vector < Hypo >& HyposMDL, Vector<int>& HypoIdx, Vector < Hypo >& HypoAll);
        double solve_mdl_exactly(Matrix<double>& Q, Vector<double>& m, Vector < Hypo >& HyposMDL, Vector<int>& HypoIdx, Vector < Hypo >& HypoAll);
        void build_mdl_matrix(Matrix<double>& Q, Vector<Hypo>& hypos, int t, double normfct);

        bool checkInsideIoU(Vector<double>& bbox1, Vector<double>& bbox2);
        bool checkBBoxOverlap(Matrix<double>& bbox1, Matrix<double>& bbox2);
    private:
        void findmax(Vector< pair< double, int > >& models, int start, double score, int *ex_steps, double *ex_a, Vector< int >& ex_bestind, double& ex_bestscore, int ex_bestsize, int ex_dim, int STEP_SIZE);
};

#endif	/* _DENNIS_MDL_H */

