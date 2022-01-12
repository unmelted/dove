
/*****************************************************************************
*                                                                            *
*                           Algebra          								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Algebra.hpp
    Author(S)       : Me Eunkyung
    Created         : 06 Jan 2022

    Description     : Algebra.hpp
    Notes           : Linear Algebra Module
*/

#include "DefData.hpp"

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>


class Algebra {
    public:
    Algebra();
    ~Algebra();    
    Dlog dl;
    
    void SetLogFilename(string name) {this->dl.SetLogFilename(name); };
    int BSplineExample();
    int BSplineTrajectory(vector<dove::Trajectory>& traj, vector<dove::Trajectory>* out, int index);
    int KalmanInOutput(dove::KALMAN* k, dove::ANALYSIS* a, double dx, double dy, int index,
        double* ndx, double* ndy);
    int KalmanInOutput(dove::KALMAN* k, dove::ANALYSIS* a, double dx, double dy, int index);
    int MedianKernel(dove::ANALYSIS* a, vector<dove::Trajectory> traj, int kernel_size);
};

