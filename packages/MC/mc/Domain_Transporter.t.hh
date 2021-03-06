//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   MC/mc/Domain_Transporter.t.hh
 * \author Thomas M. Evans
 * \date   Mon May 12 12:02:13 2014
 * \brief  Domain_Transporter template member definitions.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#ifndef MC_mc_Domain_Transporter_t_hh
#define MC_mc_Domain_Transporter_t_hh

#include <cmath>

#include "harness/DBC.hh"
#include "harness/Diagnostics.hh"
#include "geometry/Definitions.hh"
#include "Definitions.hh"
#include "Domain_Transporter.hh"

namespace profugus
{

//---------------------------------------------------------------------------//
// CONSTRUCTOR
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
template <class Geometry>
Domain_Transporter<Geometry>::Domain_Transporter()
    : d_sample_fission_sites(false)
    , d_keff(0.0)
{
}

//---------------------------------------------------------------------------//
// PUBLIC FUNCTIONS
//---------------------------------------------------------------------------//
/*!
 * \brief Set the geometry and physics classes.
 *
 * \param geometry
 * \param physics
 */
template <class Geometry>
void Domain_Transporter<Geometry>::set(SP_Geometry geometry,
                                       SP_Physics  physics)
{
    REQUIRE(geometry);
    REQUIRE(physics);

    d_geometry = geometry;
    d_physics  = physics;

    if (d_var_reduction)
    {
        d_var_reduction->set(d_geometry);
        d_var_reduction->set(d_physics);
    }
}

//---------------------------------------------------------------------------//
/*!
 * \brief Set the variance reduction.
 *
 * \param reduction
 */
template <class Geometry>
void Domain_Transporter<Geometry>::set(SP_Variance_Reduction reduction)
{
    REQUIRE(reduction);
    d_var_reduction = reduction;

    if (d_geometry)
        d_var_reduction->set(d_geometry);
    if (d_physics)
        d_var_reduction->set(d_physics);
}

//---------------------------------------------------------------------------//
/*!
 * \brief Set regular tallies.
 *
 * \param tallies
 */
template <class Geometry>
void Domain_Transporter<Geometry>::set(SP_Tallier tallies)
{
    REQUIRE(tallies);
    d_tallier = tallies;
    ENSURE(d_tallier);
}

//---------------------------------------------------------------------------//
/*!
 * \brief Set fission site sampling.
 *
 * \param fission_sites
 * \param keff
 */
template <class Geometry>
void Domain_Transporter<Geometry>::set(SP_Fission_Sites fission_sites,
                                       double           keff)
{
    // assign the container
    d_fission_sites = fission_sites;

    // initialize the sampling flag
    d_sample_fission_sites = false;

    // set the flag indicating whether fission sites should be sampled or not
    if (d_fission_sites)
    {
        d_sample_fission_sites = true;
    }

    // assign current iterate of keff
    d_keff = keff;

    // initialize the number of fission sites to 0
    d_num_fission_sites = 0;
}

//---------------------------------------------------------------------------//
/*!
 * \brief Transport a particle through the domain.
 *
 * \param particle
 * \param bank
 */
template <class Geometry>
void Domain_Transporter<Geometry>::transport(Particle_t &particle,
                                             Bank_t     &bank)
{
    REQUIRE(d_geometry);
    REQUIRE(d_physics);
    REQUIRE(d_var_reduction);
    REQUIRE(d_tallier);
    REQUIRE(particle.alive());
    REQUIRE(particle.rng().assigned());

    // particle state
    Geo_State_t &geo_state = particle.geo_state();

    // step particle through domain while particle is alive; life is relative
    // to the domain, so a particle leaving the domain would be no longer
    // alive wrt the current domain even though the particle might be alive to
    // another domain
    while (particle.alive())
    {
        // calculate distance to collision in mean-free-paths
        d_dist_mfp = -std::log(particle.rng().ran());

        // while we are hitting boundaries, continue to transport until we get
        // to the collision site
        particle.set_event(events::BOUNDARY);

        // process particles through internal boundaries until it hits a
        // collision site or leaves the domain
        //
        // >>>>>>> IMPORTANT <<<<<<<<
        // it is absolutely essential to check the geometry distance first,
        // before the distance to boundary mesh; this naturally takes care of
        // coincident boundary mesh and problem geometry surfaces (you may end
        // up with a small, negative distance to boundary mesh on the
        // subsequent step, but who cares)
        // >>>>>>>>>>>>-<<<<<<<<<<<<<
        while (particle.event() == events::BOUNDARY)
        {
            CHECK(particle.alive());
            CHECK(d_dist_mfp > 0.0);

            // total interaction cross section
            d_xs_tot = d_physics->total(physics::TOTAL, particle);
            CHECK(d_xs_tot >= 0.0);

            // sample distance to next collision
            if (d_xs_tot > 0.0)
                d_dist_col = d_dist_mfp / d_xs_tot;
            else
                d_dist_col = constants::huge;

            // initialize the distance to collision in the step selector
            d_step.initialize(d_dist_col, events::COLLISION);

            // calculate distance to next geometry boundary
            d_dist_bnd = d_geometry->distance_to_boundary(geo_state);
            d_step.submit(d_dist_bnd, events::BOUNDARY);

            // set the next event in the particle
            CHECK(d_step.tag() < events::END_EVENT);
            particle.set_event(static_cast<events::Event>(d_step.tag()));

            // path-length tallies (the actual movement of the particle will
            // take place when we process the various events)
            d_tallier->path_length(d_step.step(), particle);

            // update the mfp distance travelled
            d_dist_mfp -= d_step.step() * d_xs_tot;

            // process a particle through the geometric boundary
            if (particle.event() == events::BOUNDARY)
                process_boundary(particle, bank);
        }

        // we are done moving to a problem boundary, now process the
        // collision; the particle is actually moved from its current position
        // in each of these calls

        // process particle at a collision
        if (particle.event() == events::COLLISION)
            process_collision(particle, bank);

        // any future events go here ...
    }
}

//---------------------------------------------------------------------------//
// PRIVATE FUNCTIONS
//---------------------------------------------------------------------------//

template <class Geometry>
void Domain_Transporter<Geometry>::process_boundary(Particle_t &particle,
                                                    Bank_t     &bank)
{
    REQUIRE(particle.alive());
    REQUIRE(particle.event() == events::BOUNDARY);

    // return if not a boundary event

    // move a particle to the surface and process the event through the surface
    d_geometry->move_to_surface(particle.geo_state());

    // get the in/out state of the particle
    int state = d_geometry->boundary_state(particle.geo_state());

    // process surface tally events on non-reflecting surfaces
    if (state != geometry::REFLECT)
        d_tallier->surface(particle);

    // reflected flag
    bool reflected = false;

    // process the boundary crossing based on the geometric state of the
    // particle
    switch (state)
    {
        case geometry::OUTSIDE:
            // the particle has left the problem geometry
            particle.set_event(events::ESCAPE);
            particle.kill();

            // add a escape diagnostic
            DIAGNOSTICS_TWO(integers["geo_escape"]++);

            ENSURE(particle.event() == events::ESCAPE);
            break;

        case geometry::REFLECT:
            // the particle has hit a reflecting surface
            reflected = d_geometry->reflect(particle.geo_state());
            CHECK(reflected);

            // add a reflecting face diagnostic
            DIAGNOSTICS_TWO(integers["geo_reflect"]++);

            ENSURE(particle.event() == events::BOUNDARY);
            break;

        case geometry::INSIDE:
            // otherwise the particle is at an internal geometry boundary;
            // update the material id of the region the particle has entered
            particle.set_matid(d_geometry->matid(particle.geo_state()));

            // add variance reduction at surface crossings
            d_var_reduction->post_surface(particle, bank);

            // add a boundary crossing diagnostic
            DIAGNOSTICS_TWO(integers["geo_surface"]++);

            ENSURE(particle.event() == events::BOUNDARY);
            break;

        default:
            CHECK(0);
    }
}

//---------------------------------------------------------------------------//

template <class Geometry>
void Domain_Transporter<Geometry>::process_collision(Particle_t &particle,
                                                     Bank_t     &bank)
{
    REQUIRE(d_var_reduction);
    REQUIRE(particle.event() == events::COLLISION);

    // move the particle to the collision site
    d_geometry->move_to_point(d_step.step(), particle.geo_state());

    // sample fission sites
    if (d_sample_fission_sites)
    {
        CHECK(d_fission_sites);
        CHECK(d_keff > 0.0);
        d_num_fission_sites += d_physics->sample_fission_site(
            particle, *d_fission_sites, d_keff);
    }

    // use the physics package to process the collision
    d_physics->collide(particle, bank);

    // apply weight windows
    d_var_reduction->post_collision(particle, bank);
}

} // end namespace profugus

#endif // MC_mc_Domain_Transporter_t_hh

//---------------------------------------------------------------------------//
//                 end of Domain_Transporter.t.hh
//---------------------------------------------------------------------------//
