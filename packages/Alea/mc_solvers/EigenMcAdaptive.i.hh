//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   EigenMcAdaptive.cc
 * \author Massimiliano Lupo Pasini
 * \brief  Perform single history of MC for eigenvalue problems
 */
//---------------------------------------------------------------------------//

#ifndef mc_solver_EigenMcAdaptive_i_hh
#define mc_solver_EigenMcAdaptive_i_hh

#include <iterator>
#include <random>
#include <cmath>
#include <iomanip>

#include "EigenMcAdaptive.hh"
#include "MC_Components.hh"
#include "utils/String_Functions.hh"
#include "harness/Warnings.hh"

namespace alea
{

//---------------------------------------------------------------------------//
/*!
 * \brief Constructor
 *
 * \param P Views into entries of probability matrix
 * \param W Views into entries of weight matrix
 * \param inds Views into nonzeros indices
 * \param offsets Starting indices for each matrix row
 * \param coeffs Polynomial coefficients
 * \param pl Problem parameters
 */
//---------------------------------------------------------------------------//
EigenMcAdaptive::EigenMcAdaptive(
        Teuchos::RCP<const EigenMC_Data> mc_data,
        Teuchos::RCP<Teuchos::ParameterList> pl,
        generator_pool rand_pool)

  : d_N(mc_data->getMatrix()->getGlobalNumRows())
  , d_rand_pool(rand_pool)
  , d_rand_gen(d_rand_pool.get_state())
{
    // Get parameters
    d_max_num_histories  = pl->get("num_histories",1000);
    d_max_history_length = pl->get("max_history_length",2);
    d_weight_cutoff      = pl->get("weight_cutoff",0.0);
    d_batch_size         = pl->get("batch_size",100);
    d_tolerance          = pl->get("tolerance",0.01);

    // Determine type of tally
    std::string estimator = pl->get<std::string>("estimator",
                                                 "expected_value");
    VALIDATE(estimator == "collision" ||
             estimator == "expected_value",
             "Only collision and expected_value estimators are available.");
    d_use_expected_value = (estimator == "expected_value");

    // Should we print anything to screen
    std::string verb = profugus::to_lower(pl->get("verbosity","low"));
    if( verb == "none" )
        d_verbosity = NONE;
    else if( verb == "low" )
        d_verbosity = LOW;
    else if( verb == "high" )
        d_verbosity = HIGH;

    extractMatrices(mc_data);
}

//---------------------------------------------------------------------------//
// Solve problem using Monte Carlo
//---------------------------------------------------------------------------//
void EigenMcAdaptive::solve(const MV &b, MV &x)
{
    std::cout<<"MC eigen solver called"<<std::endl;
    Teuchos::ArrayRCP<double> x_data = x.getDataNonConst(0);
    std::fill( x_data.begin(), x_data.end(), 0.0 );

    Teuchos::ArrayRCP<unsigned int> row_null( x_data.size() );
    std::fill( row_null.begin(), row_null.end(), 0 );

    Teuchos::ArrayRCP<unsigned int> hist_count( x_data.size() );
    std::fill( hist_count.begin(), hist_count.end(), 0 );

    Teuchos::ArrayRCP<double> x_data_old( x_data.size() );
    std::fill( x_data_old.begin(), x_data_old.end(), 0.0 );

    Teuchos::ArrayRCP<double> lambda_vec( x_data.size() );
    std::fill( lambda_vec.begin(), lambda_vec.end(), 0.0 );

    // Storage for current row of A, P, W
    Teuchos::ArrayView<const double> a_row, p_row, w_row;
    Teuchos::ArrayView<const int> ind_row;

    double x_new_batch;
    double x_old_batch;
    Teuchos::ArrayRCP<double> variance(d_N);
    double variance_batch;

    Teuchos::ArrayRCP<const double> b_data = b.getData(0);

    int state = -1;
    double wt = 0.0;

    int total_histories = 0;
    double norm1_rel_std_dev = 0.0;
    double norm1_sol = 0.0;

    for (int entry=0; entry < d_N; ++entry)
    {
    	double rel_std_dev = 1e6;
    	int batch=0;
    	int num_histories = 0;
    	batch=0;
        
        std::vector<SCALAR> p_row_vec = Teuchos::createVector( d_P[entry] );
        SCALAR sum = std::accumulate(p_row_vec.begin(), p_row_vec.end(), 0.0);

        if( sum == 0.0 )
        {
            int init_wt = 0.0;
            int stage = 0 ;
	    wt = init_wt;

            // Perform initial tally
            rel_std_dev=0.0;
  	    row_null[entry] = 1;
            
        }

        else
        {
	    while( rel_std_dev > d_tolerance && num_histories < d_max_num_histories )
	    {
		 batch++;
		 x_new_batch = 0.0;
		 x_old_batch = 0.0;
		 variance_batch = 0.0;            

		 for( int i=0; i<d_batch_size; ++i )
		 {
                     double x_history_old = 0.0;
		     double x_history = 0.0;
		     int stage = 0 ;
		     int init_wt = 1.0;
		     wt = 1.0 ;

		     // Perform initial tally
		     tallyContribution(wt*b_data[entry],x_history);

		     // Get new rows for this state
		     a_row   = d_A[entry];
		     p_row   = d_P[entry];
		     w_row   = d_W[entry];
		     ind_row = d_ind[entry]; 
		     state = entry;
 
		     for( ; stage<=d_max_history_length-1; ++stage )
		     {

        		std::vector<SCALAR> p_row_vec = Teuchos::createVector( p_row );
        		SCALAR sum = std::accumulate(p_row_vec.begin(), p_row_vec.end(), 0.0);
                        if(0.0==sum)
			{
				wt = 0.0;
				break;
			}
		         
			// Move to new state
		        getNewState(state,wt,a_row,p_row,w_row,ind_row);
		             
		        // Tally
		        tallyContribution(wt*b_data[state],x_history);

		     }
		     if (0.0 != wt)
		     {
		     	x_old_batch  += x_history;
		
		     	getNewState(state,wt,a_row,p_row,w_row,ind_row);    

		     	tallyContribution(wt*b_data[state],x_history);

		     	x_new_batch  += x_history;
			hist_count[entry] = hist_count[entry] + 1;
		     }
	    	 }

		 x_data_old[entry] += x_old_batch;

		 variance_batch += x_new_batch * x_new_batch;

		 // From the old variance, compute the new second moment
		 variance[entry] = (variance[entry] * static_cast<double>(num_histories-1) +
		         x_data[entry]*x_data[entry]*static_cast<double>(num_histories) +
		         variance_batch);

		 // Compute new mean
		 x_data[entry] = (x_data[entry] * static_cast<double>(num_histories) +
		         x_new_batch) / static_cast<double>(num_histories+d_batch_size);

		 num_histories += d_batch_size;

		 variance[entry] = (variance[entry] - x_data[entry]*x_data[entry]*
		         static_cast<double>(num_histories)) /
		     static_cast<double>(num_histories-1);

		 if(x_data[entry] == 0.0)
		     rel_std_dev=1e6;
		 else
		 {
		     // Compute 1-norm of solution and variance of mean
		     double std_dev = 0;
		     double var = variance[entry] / static_cast<double>(num_histories);
		     if( var > 0.0 )
		        std_dev += std::sqrt(var);

		     rel_std_dev = static_cast<double>( std_dev / static_cast<double>(std::abs(x_data[entry])) );
		 }
		    
             }
	     x_data_old[entry] /= num_histories;
	     lambda_vec[entry] = static_cast<double>( x_data[entry]/static_cast<double>(x_data_old[entry]) );
        }
	norm1_sol += std::abs( x_data[entry] );
	norm1_rel_std_dev += rel_std_dev; 
        total_histories += num_histories;
    }

    unsigned int num_entries_valid = 0;
    double lambda = 0.0;

    for(unsigned int i=0; i!=d_N; ++i)
	if(row_null[i]==0)
	{
		lambda += lambda_vec[i];
		num_entries_valid++;
	}
	else
		x_data[i] = 0.0;

    lambda /= num_entries_valid;

    std::cout<<"Num entries valid: "<<num_entries_valid<<std::endl;

    std::cout << "Performed " << total_histories << " total histories, "
        << " average of " <<
        static_cast<double>(total_histories)/static_cast<double>(num_entries_valid)
        << " per entry" << std::endl;
    if( d_verbosity ==HIGH )
    {	
    	std::cout<<"MC estimation of the biggest eigenvalue: "<<std::setprecision(15)<<lambda<<std::endl;
	std::cout<<"norm of the relative std deviation: "<< norm1_rel_std_dev/norm1_sol <<std::endl;
    }

	
}

//---------------------------------------------------------------------------//
// PRIVATE FUNCTIONS
//---------------------------------------------------------------------------//

//---------------------------------------------------------------------------//
// Extract matrices into ArrayView objects for faster data access
//---------------------------------------------------------------------------//
void EigenMcAdaptive::extractMatrices(Teuchos::RCP<const EigenMC_Data> mc_data)
{
    d_A.resize(d_N);
    d_P.resize(d_N);
    d_W.resize(d_N);
    d_ind.resize(d_N);

    Teuchos::RCP<const MATRIX> A = mc_data->getMatrix();
    Teuchos::RCP<const MATRIX> P = mc_data->getProbabilityMatrix();
    Teuchos::RCP<const MATRIX> W = mc_data->getWeightMatrix();

    d_col_map = A->getColMap();

    Teuchos::ArrayView<const double> val_row;
    Teuchos::ArrayView<const int>    ind_row;
    for( int i=0; i<d_N; ++i )
    {
        // Extract row i of matrix
        A->getLocalRowView(i,ind_row,val_row);
        d_A[i] = val_row;
        P->getLocalRowView(i,ind_row,val_row);
        d_P[i] = val_row;
        W->getLocalRowView(i,ind_row,val_row);
        d_W[i] = val_row;
        d_ind[i] = ind_row;
    }
}

//---------------------------------------------------------------------------//
/*!
 * \brief Tally contribution into vector
 */
//---------------------------------------------------------------------------//
void EigenMcAdaptive::tallyContribution(double wt, double& x)
{
        x = wt;
}

/*!
 * \brief Get new state by sampling from cdf
 */
//---------------------------------------------------------------------------//
void EigenMcAdaptive::getNewState(int &state, double &wt,
        Teuchos::ArrayView<const double> &a_row,
        Teuchos::ArrayView<const double> &p_row,
        Teuchos::ArrayView<const double> &w_row,
        Teuchos::ArrayView<const int>    &ind_row)
{
    // Generate random number
    double rand = Kokkos::rand<generator_type,double>::draw(d_rand_gen);

    // Sample cdf to get new state
    auto elem = std::lower_bound(p_row.begin(),p_row.end(),rand);

    if( elem == p_row.end() )
    {
        // Invalidate all row data
        a_row   = Teuchos::ArrayView<const double>();
        p_row   = Teuchos::ArrayView<const double>();
        w_row   = Teuchos::ArrayView<const double>();
        ind_row = Teuchos::ArrayView<const int>();
        return;
    }
	
    // Modify weight and update state
    int index = elem - p_row.begin();
    state  =  d_col_map->getGlobalElement( ind_row[index] );
    wt    *=  w_row[index];

    // Get new rows for this state
    a_row   = d_A[state];
    p_row   = d_P[state];
    w_row   = d_W[state];
    ind_row = d_ind[state];
    
}


} // namespace alea

#endif // mc_solver_EigenMcAdaptive_i_hh