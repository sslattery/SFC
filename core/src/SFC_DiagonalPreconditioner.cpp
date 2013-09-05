//---------------------------------------------------------------------------//
/*
  Copyright (c) 2013, Stuart R. Slattery
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  *: Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  *: Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  *: Neither the name of the University of Wisconsin - Madison nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//---------------------------------------------------------------------------//
/*!
 * \file   SFC_DiagonalPreconditioner.cpp
 * \author Stuart Slattery
 * \brief  Diagonalan operator class.
 */
//---------------------------------------------------------------------------//

#include <algorithm>

#include "SFC_DBC.hpp"
#include "SFC_DiagonalPreconditioner.hpp"

#include <Teuchos_as.hpp>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
DiagonalPreconditioner::DiagonalPreconditioner( 
    const Teuchos::RCP<NonlinearProblem>& nonlinear_problem )
    : d_nonlinear_problem( nonlinear_problem )
{
    SFC_REQUIRE( Teuchos::nonnull(d_nonlinear_problem) );

    int * domain_elements;
    d_nonlinear_problem->getF()->Map().MyGlobalElementsPtr( domain_elements );
    d_domain_map = Teuchos::rcp(
        new Epetra_Map( d_nonlinear_problem->getF()->Map().NumGlobalElements(),
                        d_nonlinear_problem->getF()->Map().NumMyElements(),
                        domain_elements, 0,
                        d_nonlinear_problem->getF()->Comm() ) );

    int * range_elements;
    d_nonlinear_problem->getU()->Map().MyGlobalElementsPtr( range_elements );
    d_range_map = Teuchos::rcp(
        new Epetra_Map( d_nonlinear_problem->getU()->Map().NumGlobalElements(),
                        d_nonlinear_problem->getU()->Map().NumMyElements(),
                        range_elements, 0,
                        d_nonlinear_problem->getU()->Comm() ) );
}

//---------------------------------------------------------------------------//
/*!
 * \brief Destructor.
 */
DiagonalPreconditioner::~DiagonalPreconditioner()
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * \brief Matrix-free inverse apply operation.
 */
int DiagonalPreconditioner::ApplyInverse( const Epetra_MultiVector& X, 
                                          Epetra_MultiVector& Y ) const
{
    Teuchos::RCP<Epetra_Vector> F = d_nonlinear_problem->getF();
    for ( int i = 0; i < X.MyLength(); ++i )
    {
        if ( 0.0 != (*F)[i] )
        {
            Y[0][i] = X[0][i] / (*F)[i];
        }
        else
        {
            Y[0][i] = X[0][i];
        }
    }

    return 0;
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_DiagonalPreconditioner.cpp
//---------------------------------------------------------------------------//

