
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

    int BSpline();

};

