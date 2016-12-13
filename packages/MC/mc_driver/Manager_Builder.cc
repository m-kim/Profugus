//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   MC/mc_driver/Manager_Builder.cc
 * \author Steven Hamilton
 * \date   Wed Nov 25 11:25:01 2015
 * \brief  Manager_Builder member definitions.
 * \note   Copyright (C) 2015 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#include "Manager_Builder.hh"
#include "Manager_VTKm.hh"

#include "geometry/RTK_Geometry.hh"
#include "geometry/Mesh_Geometry.hh"

#include "Teuchos_DefaultComm.hpp"
#include "Teuchos_XMLParameterListHelpers.hpp"

namespace mc
{

auto Manager_Builder::build(const std::string &xml_file) -> SP_Manager_Base
{
    SCREEN_MSG("Reading xml file -> " << xml_file);

    // make the master parameterlist
    auto master = Teuchos::rcp(new Teuchos::ParameterList(""));

    // read the data on every domain
    auto comm = Teuchos::DefaultComm<int>::getComm();
    Teuchos::updateParametersFromXmlFileAndBroadcast(
        xml_file.c_str(), master.ptr(), *comm);

    SP_Manager_Base manager;

    // If a "CORE" db is present, we're building an RTK_Geometry
    if( master->isSublist("CORE") )
    {
        manager = std::make_shared<Manager_VTKm<profugus::Core> >();
    }
    // If "MESH" db is present, we're building a Mesh_Geometry
    else if( master->isSublist("MESH") )
    {
        manager = std::make_shared<Manager<profugus::Mesh_Geometry> >();
    }
    // No other options currently
    else
    {
        VALIDATE(false,"Either CORE or MESH db must be present.");
    }

    ENSURE( manager );

    manager->setup(master);
    return manager;
}

} // end namespace mc

//---------------------------------------------------------------------------//
//                 end of Manager_Builder.cc
//---------------------------------------------------------------------------//
