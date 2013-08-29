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
 * \file   SFC_BasicLineSearch.cpp
 * \author Stuart Slattery
 * \brief  Basic line search globalization.
 */
//---------------------------------------------------------------------------//

#include "SFC_DBC.hpp"
#include "SFC_BasicLineSearch.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
BasicLineSearch::BasicLineSearch( const Teuchos::ParameterList& parameters )
{
    d_max_iters = parameters.get<int>( "Line Search Maximum Iterations" );
    SFC_ENSURE( 0 <= d_max_iters );
}

//---------------------------------------------------------------------------//
/*!
 * \brief Set the nonlinear problem
 */
void BasicLineSearch::setNonlinearProblem( 
    const Teuchos::RCP<NonlinearProblem>& nonlinear_problem )
{
    SFC_REQUIRE( Teuchos::nonnull(nonlinear_problem) );
    d_nonlinear_problem = nonlinear_problem;
}
//---------------------------------------------------------------------------//
/*!
 * \brief Given a Newton update, apply the linesearch technique and compute a
    new update. 
*/
void BasicLineSearch::calculateUpdate( 
    const Teuchos::RCP<const Epetra_Vector>& newton_update,
    Teuchos::RCP<Epetra_Vector>& global_update )
{
    SFC_REQUIRE( Teuchos::nonnull(d_nonlinear_problem) );

    int epetra_error = 0.0;
    double F_norm = 0.0;
    double new_F_norm = 0.0;

    epetra_error = global_update->Update( 2.0, *newton_update, 0.0 );

    Teuchos::RCP<Epetra_Vector> new_F = Teuchos::rcp( 
        new Epetra_Vector(newton_update->Map()) );
    Teuchos::RCP<Epetra_Vector> new_u = Teuchos::rcp( 
        new Epetra_Vector(newton_update->Map()) );

    epetra_error = d_nonlinear_problem->getF()->Norm2( &F_norm );
    SFC_CHECK( 0 == epetra_error );

    bool complete = false;
    int num_iters = 0;

    while ( !complete && num_iters < d_max_iters )
    {
        epetra_error = global_update->Scale( 0.5 );
        SFC_CHECK( 0 == epetra_error );

        epetra_error = new_u->Update( 1.0, *(d_nonlinear_problem->getU()),
                                      1.0, *global_update, 0.0 );
        SFC_CHECK( 0 == epetra_error );

        d_nonlinear_problem->getModelEvaluator()->evaluate( new_u, new_F );

        epetra_error = new_F->Norm2( &new_F_norm );

        if ( new_F_norm < F_norm )
        {
            complete = true;
        }

        ++num_iters;
    }
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_BasicLineSearch.cpp
//---------------------------------------------------------------------------//

