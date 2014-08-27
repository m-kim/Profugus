//----------------------------------*-C++-*----------------------------------//
/*!
 * \file   solvers/Richardson.pt.cc
 * \author Thomas M. Evans
 * \date   Fri Feb 21 12:06:57 2014
 * \brief  Explicit instantiation of Richardson solver.
 * \note   Copyright (C) 2014 Oak Ridge National Laboratory, UT-Battelle, LLC.
 */
//---------------------------------------------------------------------------//

#include "Richardson.t.hh"
#include "Epetra_Operator.h"
#include "Epetra_MultiVector.h"
#include "AnasaziEpetraAdapter.hpp"
#include "AnasaziTpetraAdapter.hpp"
#include "Richardson.t.hh"

#include "TpetraTypedefs.hh"

namespace profugus
{

template class Richardson<Epetra_MultiVector,Epetra_Operator>;
template class Richardson<Tpetra_MultiVector,Tpetra_Operator>;

} // end namespace profugus

//---------------------------------------------------------------------------//
//                 end of Richardson.pt.cc
//---------------------------------------------------------------------------//
