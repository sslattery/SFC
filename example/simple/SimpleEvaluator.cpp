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
 * \file   SimpleEvaluator.cpp
 * \author Stuart Slattery
 * \brief  Simple model evaluator for example problem.
 */
//---------------------------------------------------------------------------//

#include <cmath>

#include "SimpleEvaluator.hpp"

#include <SFC_DBC.hpp>

namespace SimpleExample
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
SimpleEvaluator::SimpleEvaluator( 
    const double a, const double b, const double c )
    : d_a( a )
    , d_b( b )
    , d_c( c )
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * \brief Given a vector, evaluate the model and generate a nonlinear
 * residual.
 */
void SimpleEvaluator::evaluate( const Teuchos::RCP<Epetra_Vector>& u,
                                Teuchos::RCP<Epetra_Vector>& F )
{
    SFC_REQUIRE( Teuchos::nonnull(u) );
    SFC_REQUIRE( Teuchos::nonnull(F) );

    // Central points.
    for ( int i = 1; i < u->MyLength()-1; ++i )
    {
        (*F)[i] = -d_a * ( (*u)[i+1] - 2*(*u)[i] + (*u)[i-1] ) 
		  + d_b * (*u)[i]
		  - d_c;
    }

    // Boundary points - vacuum conditions.
    (*F)[0] = -d_a * ( (*u)[1] - 2*(*u)[0] )
	      + d_b * (*u)[0]
	      - d_c;

    (*F)[u->MyLength()-1] = 
	-d_a * ( - 2*(*u)[u->MyLength()-1] + (*u)[u->MyLength()-2] )
	+ d_b * (*u)[u->MyLength()-1]
	- d_c;
}

//---------------------------------------------------------------------------//

} // end namespace SimpleExample

//---------------------------------------------------------------------------//
// end SimpleEvaluator.cpp
//---------------------------------------------------------------------------//

