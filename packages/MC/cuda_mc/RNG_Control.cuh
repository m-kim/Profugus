//---------------------------------*-C++-*-----------------------------------//
/*!
 * \file   MC/cuda_mc/RNG_Control.cuh
 * \author Steven Hamilton
 * \date   Thu Mar 31 09:46:56 2016
 * \brief  RNG_Control class declaration.
 * \note   Copyright (c) 2016 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#ifndef MC_cuda_mc_RNG_Control_cuh
#define MC_cuda_mc_RNG_Control_cuh

#include <random>
#include <limits>
#include <curand_kernel.h>
#include <thrust/device_vector.h>

#include "harness/DBC.hh"

namespace cuda_mc
{

//===========================================================================//
/*!
 * \class RNG_Control
 * \brief Manage creation and seeding of RNG states.
 *
 * The use of cuRAND random number generators requires careful managing of
 * the initialization and lifetimes of the RNG states to guarantee performance
 * and independence of RNG streams.  For example, cuRAND documentation 
 * strongly recommends seeding every thread with the same seed but different
 * "stream ids" to maintain independence of the RNGs.  However, this choice
 * leads to lengthy initialization time for several generators (including
 * the default).  Instead, we randomly generate seeds on the host and provide
 * these to the cuRAND generators.
 *
 * This class also manages the allocation of RNG states during subsequent
 * cycles by keeping the states persistent and resizing the array of states
 * (and seeding them) appropriately.  This guarantees that RNGs generated
 * in one cycle are independent of other cycles.
 *
 * \sa RNG_Control.cc for detailed descriptions.
 */
//===========================================================================//

class RNG_Control
{
  public:

      typedef curandState_t                         RNG_State_t;
      typedef unsigned long long                    seed_type;
      typedef std::numeric_limits<seed_type>        limits;
      typedef thrust::device_vector<RNG_State_t>    RNG_Vector;

      //!\brief Constructor.
      RNG_Control( int host_seed )
          : d_gen(host_seed)
          , d_dist(limits::min(),limits::max())
      {
      }

      /*!\brief Initialize for specified number of RNG streams.
       * 
       * New states will only be initialized if the requested number
       * exceeds the maximum number of states from previous requests.
       */
      void initialize( int num_streams );

      /*!\brief Get device pointer to current states.
       *
       * The number of valid states is guaranteed to be at least as large
       * as the most recent call to initialize.
       */
      RNG_Vector & get_states()
      {
          return d_rng_states;
      }

  private:

      // Host-side random number generator
      std::mt19937 d_gen;
      std::uniform_int_distribution<seed_type> d_dist;

      // Device storage for RNG states
      thrust::device_vector<RNG_State_t> d_rng_states;
};

//---------------------------------------------------------------------------//
} // end namespace cuda_mc

//---------------------------------------------------------------------------//
#endif // MC_cuda_mc_RNG_Control_cuh

//---------------------------------------------------------------------------//
// end of MC/cuda_mc/RNG_Control.cuh
//---------------------------------------------------------------------------//
