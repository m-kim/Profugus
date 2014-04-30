//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   mc/Particle.hh
 * \author Thomas M. Evans
 * \date   Fri Apr 25 11:26:16 2014
 * \brief  Particle class definition.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#ifndef mc_Particle_hh
#define mc_Particle_hh

#include "utils/Definitions.hh"
#include "rng/RNG.hh"
#include "Definitions.hh"

namespace profugus
{

//===========================================================================//
/*!
 * \class Particle
 * \brief Particle class for MC transport.
 */
/*!
 * \example mc/test/tstParticle.cc
 *
 * Test of Particle.
 */
//===========================================================================//

class Particle
{
  public:
    //@{
    //! Typedefs.
    typedef def::Space_Vector Space_Vector;
    typedef events::Event     Event_Type;
    //@}

  private:
    // >>> DATA

    // Material id in current region.
    int d_matid;

    // Particle weight.
    double d_wt;

    // Random number generator (reference counted).
    RNG d_rng;

    // Alive/dead status.
    bool d_alive;

    // Latest particle event.
    Event_Type d_event;

  public:
    // Constructor
    Particle();

    // >>> PARTICLE FUNCTIONS

    //! Set a new weight.
    void set_wt(double wt) { d_wt = wt; }

    //! Multiply weight.
    void multiply_wt(double wt) { d_wt *= wt; }

    //! Set a new random number generator.
    void set_rng(const RNG &rng) { d_rng = rng; }

    //! Set the particle event flag.
    void set_event(Event_Type event) { d_event = event; }

    //! Set the material id of the region occupied by the particle.
    void set_matid(int matid) { d_matid = matid; }

    //! Kill the particle.
    void kill() { d_alive = false; }

    //! Set particle status to alive.
    void live() { d_alive = true; }

    //@{
    //! Access particle data.
    bool alive() const { return d_alive; }
    double wt() const { return d_wt; }
    const RNG& rng() const { return d_rng; }
    Event_Type event() const { return d_event; }
    int matid() const { return d_matid; }
    //@}
};

} // end namespace profugus

#endif // mc_Particle_hh

//---------------------------------------------------------------------------//
//                 end of Particle.hh
//---------------------------------------------------------------------------//