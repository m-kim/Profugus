//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   mc/Source.cc
 * \author Thomas M. Evans
 * \date   Mon May 05 14:28:41 2014
 * \brief  Source member definitions.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#include "Source.hh"
#include "harness/DBC.hh"

namespace profugus
{

//---------------------------------------------------------------------------//
// CONSTRUCTOR/DESTRUCTOR
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
Source::Source(SP_Geometry    geometry,
               SP_Physics     physics,
               SP_RNG_Control rng_control)
    : b_geometry(geometry)
    , b_physics(physics)
    , b_rng_control(rng_control)
{
    Require (b_geometry);
    Require (b_physics);
    Require (b_rng_control);
}

//---------------------------------------------------------------------------//
/*!
 * \brief Virtual destructor.
 */
Source::~Source()
{
}

} // end namespace profugus

//---------------------------------------------------------------------------//
//                 end of Source.cc
//---------------------------------------------------------------------------//