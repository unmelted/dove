
/*****************************************************************************
*                                                                            *
*                           Algebra          								 *
*                                                                            *
*   Copyright (C) 2021 By 4dreplay, Incoporated. All Rights Reserved.        *
******************************************************************************

    File Name       : Algebra.cpp
    Author(S)       : Me Eunkyung
    Created         : 06 Jan 2022

    Description     : Algebra.cpp
    Notes           : Linear Algebra Module
*/

#include "Algebra.hpp"

Algebra::Algebra() {
    dl = Dlog();
}

Algebra::~Algebra() 
{

}

int Algebra::PostProcess(vector<dove::FRAME_INFO>& info, dove::ANALYSIS* a) {

}

int Algebra::BSplineTrajectory(vector<dove::Trajectory>& gt, vector<dove::Trajectory>* out, int index) {
    const int NCOEFFS = 4;
    /* nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 */
    const int  NBREAK = (NCOEFFS - 2);

    ofstream out_;
    if (index == 0 )
        out_.open("analysis/spline_x.txt");
    else 
        out_.open("analysis/spline_y.txt");

    const size_t n = gt.size();
    dl.Logger(" n size %d", n);

    const size_t ncoeffs = NCOEFFS;
    const size_t nbreak = NBREAK;
    size_t i, j;
    gsl_bspline_workspace *bw;
    gsl_vector *B;
    double dy;
    gsl_rng *r;
    gsl_vector *c, *w;
    gsl_vector *x, *y;
    gsl_matrix *X, *cov;
    gsl_multifit_linear_workspace *mw;
    double chisq, Rsq, dof, tss;

    gsl_rng_env_setup();
    r = gsl_rng_alloc(gsl_rng_default);

    bw = gsl_bspline_alloc(4, nbreak);
    B = gsl_vector_alloc(ncoeffs);

    x = gsl_vector_alloc(n);
    y = gsl_vector_alloc(n);
    X = gsl_matrix_alloc(n, ncoeffs);
    c = gsl_vector_alloc(ncoeffs);
    w = gsl_vector_alloc(n);
    cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
    mw = gsl_multifit_linear_alloc(n, ncoeffs);

    for (int i = 0 ; i < n; i++) {
        double det;
        if (index == 0 ) { 
            // if( n > 0 ) 
            //     det = ((gt[i-1].x - gt[i].x));
            // else
            //     det = ( gt[i].x * 0.1);
            gsl_vector_set(y, i, gt[i].x);     
        } else {
            gsl_vector_set(y, i, gt[i].y);
        }
        gsl_vector_set(x, i, i);
        gsl_vector_set(w, i, 1);   
    }

    gsl_bspline_knots_uniform(0.0, n-1, bw);

    for (i = 0; i < n; ++i) {

        double xi = gsl_vector_get(x, i);
        gsl_bspline_eval(xi, B, bw);
        for (j = 0; j < ncoeffs; ++j) {
            double Bj = gsl_vector_get(B, j);
            gsl_matrix_set(X, i, j, Bj);
        }
    }

    gsl_multifit_wlinear(X, w, y, c, cov, &chisq, mw);
    dof = n - ncoeffs;
    dl.Logger("chisq %f %e", chisq, chisq / dof);
    double xi, yi, yerr, origin;
    for(xi = 0 ; xi < n ; xi++) { 
        gsl_bspline_eval(xi, B, bw);
        gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
        out->push_back(dove::Trajectory(0, yi, 0));        
        out_ << xi << " "<< yi << endl;
//        printf(" %f %f \n", xi, yi);

    }
    gsl_rng_free(r);
    gsl_bspline_free(bw);
    gsl_vector_free(B);
    gsl_vector_free(x);
    gsl_vector_free(y);
    gsl_matrix_free(X);
    gsl_vector_free(c);
    gsl_vector_free(w);
    gsl_matrix_free(cov);
    gsl_multifit_linear_free(mw);

    return 1;
}

int Algebra::BSplineExample() {
/* number of data points to fit */
#define N        200
/* number of fit coefficients */
#define NCOEFFS  4
/* nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 */
#define NBREAK   (NCOEFFS - 2)
    
    const size_t n = N;
    const size_t ncoeffs = NCOEFFS;
    const size_t nbreak = NBREAK;
    size_t i, j;
    gsl_bspline_workspace *bw;
    gsl_vector *B;
    double dy;
    gsl_rng *r;
    gsl_vector *c, *w;
    gsl_vector *x, *y;
    gsl_matrix *X, *cov;
    gsl_multifit_linear_workspace *mw;
    double chisq, Rsq, dof, tss;

    gsl_rng_env_setup();
    r = gsl_rng_alloc(gsl_rng_default);

    bw = gsl_bspline_alloc(4, nbreak);
    B = gsl_vector_alloc(ncoeffs);

    x = gsl_vector_alloc(n);
    y = gsl_vector_alloc(n);
    X = gsl_matrix_alloc(n, ncoeffs);
    c = gsl_vector_alloc(ncoeffs);
    w = gsl_vector_alloc(n);
    cov = gsl_matrix_alloc(ncoeffs, ncoeffs);
    mw = gsl_multifit_linear_alloc(n, ncoeffs);

    for (i = 0; i < n; ++i)
    {
        double sigma;
        double xi = (15.0 / (N - 1)) * i;
        double yi = cos(xi) * exp(-0.1 * xi);

        sigma = 0.1 * yi;
        dy = gsl_ran_gaussian(r, sigma);
        yi += dy;

        gsl_vector_set(x, i, xi);
        gsl_vector_set(y, i, yi);
        gsl_vector_set(w, i, 1.0 / (sigma * sigma));

        printf("%f %f\n", xi, yi);
    }

    gsl_bspline_knots_uniform(0.0, 15.0, bw);

    for (i = 0; i < n; ++i)
    {
        double xi = gsl_vector_get(x, i);

        gsl_bspline_eval(xi, B, bw);

        for (j = 0; j < ncoeffs; ++j)
        {
            double Bj = gsl_vector_get(B, j);
            gsl_matrix_set(X, i, j, Bj);
        }
    }

    gsl_multifit_wlinear(X, w, y, c, cov, &chisq, mw);

    dof = n - ncoeffs;
    tss = gsl_stats_wtss(w->data, 1, y->data, 1, y->size);
    Rsq = 1.0 - chisq / tss;

    fprintf(stderr, "chisq/dof = %e, Rsq = %f\n",
                chisq / dof, Rsq);

    printf("\n\n");

    {
        double xi, yi, yerr;

        for (xi = 0.0; xi < 15.0; xi += 0.1) {
            gsl_bspline_eval(xi, B, bw);
            gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
            printf("%f %f\n", xi, yi);
        }
    }

    gsl_rng_free(r);
    gsl_bspline_free(bw);
    gsl_vector_free(B);
    gsl_vector_free(x);
    gsl_vector_free(y);
    gsl_matrix_free(X);
    gsl_vector_free(c);
    gsl_vector_free(w);
    gsl_matrix_free(cov);
    gsl_multifit_linear_free(mw);

    return 0;    
}

int Algebra::KalmanInOutput(dove::KALMAN* k, dove::ANALYSIS* a, double dx, double dy, int index,
        double* ndx, double* ndy) {

    double da = 0;
    k->x += dx;
    k->y += dy;
    k->a += da;
    a->out_transform << index << " " << dx << " " << dy << " " << da << endl;

    k->z = dove::Trajectory(k->x, k->y, k->a);
    if( index == 0 ){
        k->X = dove::Trajectory(0,0,0); //Initial estimate,  set 0
        k->P = dove::Trajectory(1,1,1); //set error variance,set 1
    }
    else
    {
        //time update（prediction）
        k->X_ = k->X; //X_(k) = X(k-1);
        k->P_ = k->P+ k->Q; //P_(k) = P(k-1)+Q;
        // measurement update（correction）
        k->K = k->P_/ ( k->P_+ k->R ); //gain;K(k) = P_(k)/( P_(k)+R );
        k->X = k->X_+ k->K * (k->z - k->X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
        k->P = (dove::Trajectory(1,1,1) - k->K) * k->P_; //P(k) = (1-K(k))*P_(k);
    }
    //smoothed_trajectory.push_back(X);
    a->out_smoothed << index << " " << k->X.x << " " << k->X.y << " " << k->X.a << endl;

    // target - current
    double diff_x = k->X.x - k->x;//
    double diff_y = k->X.y - k->y;
    double diff_a = k->X.a - k->a;

    *ndx = dx + diff_x;
    *ndy = dy + diff_y;
    da = da + diff_a;
    dl.Logger("pre from kalman %f %f ", *ndx, *ndy);
    a->out_new << index << " " << *ndx << " " << *ndy << " " << da << endl;
    return dove::ERR_NONE;
}

int Algebra::KalmanInOutput(dove::KALMAN* k, dove::ANALYSIS* a, double dx, double dy, int index,
        vector<dove::TransformParam>* out) {
    double new_dx = 0;
    double new_dy = 0;
    KalmanInOutput(k, a, dx, dy, index, &new_dx, &new_dy );
    out->push_back(dove::TransformParam(new_dx, new_dy, 0));
    return dove::ERR_NONE;
}

int Algebra::MedianKernel(dove::ANALYSIS* a, vector<dove::Trajectory> traj, int kernel_size, vector<dove::Trajectory>* out) {
    for(size_t i = 0; i < traj.size(); i++) {
        double sum_x = 0;
        double sum_y = 0;
        double sum_a = 0;
        int count = 0;

        for(int j = -kernel_size; j <= kernel_size; j++) {
            if(i+j >= 0 && i+j < traj.size()) {
                sum_x += traj[i+j].x;
                sum_y += traj[i+j].y;
                sum_a += traj[i+j].a;

                count++;
            }
        }
        double avg_a = sum_a / count;
        double avg_x = sum_x / count;
        double avg_y = sum_y / count;
        out->push_back(dove::Trajectory(avg_x, avg_y, avg_a));
        a->out_smoothed << (i+1) << " " << avg_x << " " << avg_y << " " << "0" << endl;
    }
    return dove::ERR_NONE;
}