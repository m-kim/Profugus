//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   mc/Uniform_Source.cc
 * \author Thomas M. Evans
 * \date   Tue May 06 16:43:26 2014
 * \brief  Uniform_Source member definitions.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#include <numeric>

#include "Teuchos_Array.hpp"

#include "harness/Soft_Equivalence.hh"
#include "harness/DBC.hh"
#include "comm/Timing.hh"
#include "comm/global.hh"
#include "Sampler.hh"
#include "Uniform_Source.hh"
#include "rng/RNG.hh"

namespace profugus
{

//---------------------------------------------------------------------------//
// CONSTRUCTOR
//---------------------------------------------------------------------------//
/* 
 * \brief Default constructor.
 */
Uniform_Source::Uniform_Source()
    : d_node( profugus::node() )
    , d_nodes( profugus::nodes() )
    , d_wt(1.0)
{
}

//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 *
 * \param db
 * \param geometry
 * \param physics
 * \param rng
 */
Uniform_Source::Uniform_Source(RCP_Std_DB     db,
			       SP_Geometry    geometry,
			       SP_Physics     physics,
			       SP_Shape geometric_shape )
    : d_geometry( geometry )
    , d_physics( physics )
    , d_geo_shape( geometric_shape )
    , d_erg_cdf(d_physics->num_groups(), 0.0)
    , d_node( profugus::node() )
    , d_nodes( profugus::nodes() )
    , d_np_requested(0)
    , d_np_total(0)
    , d_np_domain(0)
    , d_wt(1.0)
{
    REQUIRE(!db.is_null());

    // store the total number of requested particles
    d_np_requested = static_cast<size_type>(db->get("Np", 1000));
    VALIDATE(d_np_requested > 0., "Number of source particles ("
            << d_np_requested << ") must be positive");

    // initialize the total
    d_np_total = d_np_requested;

    // get the spectral shape
    const auto &shape = db->get(
        "spectral_shape", Teuchos::Array<double>(d_physics->num_groups(), 1.0));
    CHECK(shape.size() == d_erg_cdf.size());

    // calculate the normalization
    double norm = std::accumulate(shape.begin(), shape.end(), 0.0);
    CHECK(norm > 0.0);

    // assign to the shape cdf
    norm  = 1.0 / norm;
    d_erg_cdf[0] = shape[0] * norm;
    for ( int n = 1; n < d_erg_cdf.size(); ++n )
    {
        d_erg_cdf[n] = d_erg_cdf[n-1] + shape[n] * norm;
    }
    CHECK(profugus::soft_equiv(1.0, d_erg_cdf.back(), 1.0e-6));

    // build the source based on domain replication
    build_DR();

    // Get the batch size.
    d_batch_size = static_cast<size_type>(db->get("Batch Size",d_np_domain));
    REQUIRE( d_batch_size <= d_np_domain );

    profugus::global_barrier();
}

//---------------------------------------------------------------------------//
// PUBLIC FUNCTIONS
//---------------------------------------------------------------------------//
/*!
 * \brief Get a particle from the source given a particle local id.
 */
Uniform_Source::Particle_t Uniform_Source::get_particle( const int lid )
{
    using def::I; using def::J; using def::K;

    REQUIRE(d_wt > 0.0);
    REQUIRE(d_geo_shape);

    // throw if no source.
    REQUIRE(lid <= d_np_domain);

    // make a particle
    Particle_t p;

    // create a unique rng for the particle
    int rng_seed = lid*d_nodes + d_node;
    p.set_rng( RNG(rng_seed) );

    // material id
    int matid = 0;

    // sample the angle isotropically
    Space_Vector omega = sample_angle(p.rng().ran(), p.rng().ran() );

    // sample the geometry shape-->we should not get here if there are no
    // particles on this domain
    Space_Vector r = d_geo_shape->sample(p.rng().ran(),p.rng().ran(),p.rng().ran());

    // intialize the geometry state
    d_geometry->initialize(r, omega, p.geo_state());

    // get the material id
    matid = d_geometry->matid(p.geo_state());

    // initialize the physics state by manually sampling the group
    int group = sampler::sample_discrete_CDF(
        d_erg_cdf.size(), &d_erg_cdf[0], p.rng().ran());
    CHECK(group < d_physics->num_groups());

    // set the group
    p.set_group(group);

    // set the material id in the particle
    p.set_matid(matid);

    // set particle weight
    p.set_wt(d_wt);

    // make particle alive
    p.live();

    ENSURE(p.matid() == matid);
    return p;
}

//---------------------------------------------------------------------------//
// PRIVATE IMPLEMENTATION
//---------------------------------------------------------------------------//
/*!
 * \brief Build domain replicated source.
 */
void Uniform_Source::build_DR()
{
    // calculate the number of particles per domain and set (equivalent)
    d_np_domain = d_np_total / d_nodes;

    // recalculate the total number of particles (we want the same number of
    // particles in each domain, so the total may change slightly from the
    // requested value)
    d_np_total  = d_np_domain * d_nodes;
}

} // end namespace profugus

//---------------------------------------------------------------------------//
//                 end of Uniform_Source.cc
//---------------------------------------------------------------------------//