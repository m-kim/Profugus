//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   EigenMcAdaptive.hh
 * \author Massimiliano Lupo Pasini
 * \brief  Perform MC histories to solve eigenvalue problems.
 */
//---------------------------------------------------------------------------//

#ifndef mc_solver_EigenMcAdaptive_hh
#define mc_solver_EigenMcAdaptive_hh

#include "MC_Data.hh"

#include "AleaTypedefs.hh"

namespace alea
{

//---------------------------------------------------------------------------//
/*!
 * \class EigenMcAdaptive
 * \brief Perform Monte Carlo random walks on linear system.
 *
 * This class performs random walks using the forward Monte Carlo algorithm
 * with an adaptive stopping criterion based on variance of the mean.
 */
//---------------------------------------------------------------------------//

class EigenMcAdaptive
{
  public:

    typedef Kokkos::Random_XorShift64_Pool<DEVICE>  generator_pool;
    typedef typename generator_pool::generator_type generator_type;

    EigenMcAdaptive(Teuchos::RCP<const EigenMC_Data>       mc_data,
                      Teuchos::RCP<Teuchos::ParameterList> pl,
                      generator_pool                       rand_pool);

    //! Solve problem
    void solve(const MV &b, MV &x);

  private:

    // Convert from Tpetra matrices to ArrayViews
    void extractMatrices(Teuchos::RCP<const EigenMC_Data> mc_data);

    // Transition a history to a new state
    inline void getNewState(int &state, double &wt,
        Teuchos::ArrayView<const double> &a_row,
        Teuchos::ArrayView<const double> &p_row,
        Teuchos::ArrayView<const double> &w_row,
        Teuchos::ArrayView<const int>    &ind_row);

    // Add contribution of current history to solution
    inline void tallyContribution(double wt,double& x);

    // Data for Monte Carlo
    Teuchos::ArrayRCP<Teuchos::ArrayView<const double> > d_A;
    Teuchos::ArrayRCP<Teuchos::ArrayView<const double> > d_P;
    Teuchos::ArrayRCP<Teuchos::ArrayView<const double> > d_W;
    Teuchos::ArrayRCP<Teuchos::ArrayView<const int> >    d_ind;

    //storage of the column map
    Teuchos::RCP<const MAP> d_col_map;

    // Vector length
    int d_N;

    // Problem parameters
    int    d_max_history_length;
    bool   d_use_expected_value;
    int    d_max_num_histories;
    double d_weight_cutoff;
    int    d_batch_size;
    double d_tolerance;

    enum VERBOSITY {NONE, LOW, HIGH};
    VERBOSITY d_verbosity;

    // Kokkos random number generator
    generator_pool d_rand_pool;
    generator_type d_rand_gen;
};

} // namespace alea

#include "EigenMcAdaptive.i.hh"

#endif // mc_solver_EigenMcAdaptive_hh
