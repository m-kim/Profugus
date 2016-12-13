//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   MC/mc_driver/Manager_VTKm.pt.cc
 * \author Steven Hamilton
 * \date   Wed Nov 25 11:42:29 2015
 * \brief  Manager template instantiations
 * \note   Copyright (C) 2015 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#include "Manager_VTKm.hh"
#include "geometry/RTK_Geometry.hh"
#include "geometry/Mesh_Geometry.hh"

namespace mc
{

template class Manager_VTKm<profugus::Core>;
template class Manager_VTKm<profugus::Mesh_Geometry>;

} // end namespace mc

//---------------------------------------------------------------------------//
//                 end of Manager_VTKm.pt.cc
//---------------------------------------------------------------------------//
