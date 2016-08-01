//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   mc/VR_Roulette.t.hh
 * \author Thomas M. Evans
 * \date   Fri May 09 13:09:37 2014
 * \brief  VR_Roulette member definitions.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#ifndef mc_VR_Roulette_t_cuh
#define mc_VR_Roulette_t_cuh

#include "VR_Roulette.hh"
#include "Definitions.hh"

#include "cuda_utils/Hardware.hh"
#include "cuda_utils/CudaDBC.hh"

namespace cuda_profugus
{
//---------------------------------------------------------------------------//
// CUDA KERNELS
//---------------------------------------------------------------------------//
template<class Geometry>
__global__ void post_collision_kernel( const int num_collision,
				       const double w_c,
				       const double w_s,
				       Particle_Vector<Geometry>* particles )
{
    // Get the thread index.
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int collision_start = particles->event_lower_bound( events::COLLISION );

    if ( idx < num_collision )
    {
	// Get the particle index.
	int pidx = idx + collision_start;

	if ( particles->alive(pidx) )
	{
	    // get the particle weight
	    const double orig_weight = particles->wt(pidx);

	    // if the particle weight is below the cutoff do roulette
	    if (orig_weight < w_c)
	    {
		// calculate survival probablity
		CHECK(w_s >= w_c);
		const double survival = orig_weight / w_s;
		CHECK(survival < 1.0);

		// particle survives roulette
		if (particles->ran(pidx) < survival)
		{
		    // set the new weight of the surviving particle
		    particles->set_wt(pidx,w_s);
		}

		// otherwise the particle dies
		else
		{
		    // kill the particle
		    particles->kill(pidx);
		}
	    }

	    ENSURE(particles->wt(pidx) >= orig_weight);
	}
    }
}

//---------------------------------------------------------------------------//
// CONSTRUCTOR
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
template <class Geometry>
VR_Roulette<Geometry>::VR_Roulette( ParameterList_t& db)
    : Base()
{
    d_Wc = db.get("weight_cutoff", 0.25);
    d_Ws = db.get("weight_survival", 2.0 * d_Wc);
    b_splitting = false;
}

//---------------------------------------------------------------------------//
// VARIANCE REDUCTION FUNCTIONS
//---------------------------------------------------------------------------//
/*!
 * \brief Process a particle for weight roulette.
 */
template <class Geometry>
void VR_Roulette<Geometry>::post_collision(
    cuda::Shared_Device_Ptr<Particle_Vector_t>& particles, 
    cuda::Shared_Device_Ptr<Bank_t>& bank,
    cuda::Stream<cuda::arch::Device> stream ) const
{
    // Get the particles that have had a collision.
    int num_particle =
        particles.get_host_ptr()->get_event_size( events::COLLISION );

    // Get CUDA launch parameters.
    REQUIRE( cuda::Hardware<cuda::arch::Device>::have_acquired() );
    unsigned int threads_per_block = 
	cuda::Hardware<cuda::arch::Device>::default_block_size();
    unsigned int num_blocks = num_particle / threads_per_block;
    if ( num_particle % threads_per_block > 0 ) ++num_blocks;

    // do roulette
    post_collision_kernel<<<num_blocks,threads_per_block,0,stream.handle()>>>( 
	num_particle,
	d_Wc,
	d_Ws,
	particles.get_device_ptr() );
}

//---------------------------------------------------------------------------//

} // end namespace cuda_profugus

#endif // mc_VR_Roulette_t_cuh

//---------------------------------------------------------------------------//
//                 end of VR_Roulette.t.hh
//---------------------------------------------------------------------------//
