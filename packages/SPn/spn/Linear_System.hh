//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   SPn/spn/Linear_System.hh
 * \author Thomas M. Evans
 * \date   Sun Oct 28 18:37:01 2012
 * \brief  Linear_System class definition.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#ifndef SPn_spn_Linear_System_hh
#define SPn_spn_Linear_System_hh

#include <string>
#include <SPn/config.h>

// Trilinos Includes
#include "Teuchos_RCP.hpp"
#include "Teuchos_Array.hpp"

#include "mesh/Mesh.hh"
#include "mesh/LG_Indexer.hh"
#include "mesh/Global_Mesh_Data.hh"
#include "solvers/LinAlgTypedefs.hh"
#include "Isotropic_Source.hh"
#include "Moment_Coefficients.hh"

namespace profugus
{

//===========================================================================//
/*!
 * \class Linear_System
 * \brief Base class that defines the Linear System for an SPN problem.
 *
 * This class builds the SPN linear system for both fixed source:
 * \f[
   \mathbf{A}\mathbf{u} = \mathbf{Q}\:,
 * \f]
 * and eigenvalue problems:
 * \f[
   \mathbf{A}\mathbf{u} = \frac{1}{k}\mathbf{B}\mathbf{u}\:.
 * \f]
 * The operators \b A and \b B along with vectors \b u and \b Q are defined in
 * the SPN technical note and Denovo methods manual.
 */
//===========================================================================//

template <class T>
class Linear_System
{
  public:
    //@{
    //! Typedefs.
    typedef Moment_Coefficients::Serial_Matrix     Serial_Matrix;
    typedef Moment_Coefficients::RCP_Mat_DB        RCP_Mat_DB;
    typedef Moment_Coefficients::RCP_Dimensions    RCP_Dimensions;
    typedef Moment_Coefficients::RCP_ParameterList RCP_ParameterList;
    typedef Moment_Coefficients::RCP_Timestep      RCP_Timestep;
    typedef Teuchos::RCP<Moment_Coefficients>      RCP_Moment_Coefficients;
    typedef typename T::MAP                        Map_t;
    typedef typename T::MATRIX                     Matrix_t;
    typedef typename T::OP                         Operator_t;
    typedef typename T::MV                         MV;
    typedef Teuchos::RCP<Map_t>                    RCP_Map;
    typedef Teuchos::RCP<Matrix_t>                 RCP_Matrix;
    typedef Teuchos::RCP<Operator_t>               RCP_Operator;
    typedef Teuchos::RCP<MV>                       RCP_MV;
    typedef Isotropic_Source                       External_Source;
    typedef Teuchos::RCP<Mesh>                     RCP_Mesh;
    typedef Teuchos::RCP<LG_Indexer>               RCP_Indexer;
    typedef Teuchos::RCP<Global_Mesh_Data>         RCP_Global_Data;
    typedef Teuchos::Array<int>                    Array_Int;
    typedef Teuchos::Array<double>                 Array_Dbl;
    //@}

  protected:
    // >>> SHARED DATA

    // Problem database.
    RCP_ParameterList b_db;

    // SPN dimensions.
    RCP_Dimensions b_dim;

    // Material database.
    RCP_Mat_DB b_mat;

    // Timestep.
    RCP_Timestep b_dt;

    // Moment-coefficient generator.
    RCP_Moment_Coefficients b_mom_coeff;

    // Trilinos objects.
    RCP_Map      b_map;
    RCP_Operator b_operator; // SPN matrix
    RCP_Operator b_fission;  // Fission matrix
    RCP_MV       b_rhs;

    // Adjoint objects (may be null).
    RCP_Operator b_adjoint_operator;
    RCP_Operator b_adjoint_fission;

    // Isotropic source coefficients.
    double b_src_coefficients[4];

    // Isotropic boundary source coefficients.
    double b_bnd_coefficients[4];

    // Adjoint flag -> if true then get_ operations return transposed
    // operators.
    bool b_adjoint;

    // Processor nodes.
    int b_node, b_nodes;

  public:
    // Constructor.
    Linear_System(RCP_ParameterList db, RCP_Dimensions dim, RCP_Mat_DB mat,
                  RCP_Timestep dt);

    // Destructor.
    virtual ~Linear_System() = 0;

    //! Make the matrix.
    virtual void build_Matrix() = 0;

    //! Build the right-hand-side from an external, isotropic source.
    virtual void build_RHS(const External_Source &q) = 0;

    //! Build the right-hand-side fission matrix.
    virtual void build_fission_matrix() = 0;

    //! Set adjoint flag.
    void set_adjoint(bool adjoint);

    // >>> ACCESSORS

    //! Get an RCP to the communication map.
    RCP_Map get_Map() const { return b_map; }

    //! Get an RCP to the full LHS Operator.
    RCP_Operator get_Operator() const
    {
        return b_adjoint ? b_adjoint_operator : b_operator;
    }

    //! Get an RCP to the LHS matrix (may not be full matrix)
    virtual RCP_Matrix get_Matrix() const { return Teuchos::null; }

    //! Get an RCP to the Right-Hand-Side vector.
    RCP_MV get_RHS() const { return b_rhs; }

    //! Get an RCP to the right-hand-side fission matrix.
    RCP_Operator get_fission_matrix() const
    {
        return b_adjoint ? b_adjoint_fission : b_fission;
    }

    //! Get problem dimensions.
    RCP_Dimensions get_dims() const { return b_dim; }

    //! Get the local/global index for (group, eqn, spatial_unknown).
    virtual int index(int g, int eqn, int spatial_unknown) const = 0;
};

//---------------------------------------------------------------------------//
// SPECIALIZATIONS
//---------------------------------------------------------------------------//

template<>
void Linear_System<EpetraTypes>::set_adjoint(bool adjoint);

}                               // end namespace profugus

#endif // SPn_spn_Linear_System_hh

//---------------------------------------------------------------------------//
//              end of spn/Linear_System.hh
//---------------------------------------------------------------------------//
