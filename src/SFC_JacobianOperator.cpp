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
 * \file   SFC_JacobianOperator.cpp
 * \author Stuart Slattery
 * \brief  Jacobian operator class.
 */
//---------------------------------------------------------------------------//

#include "SFC_DBC.hpp"
#include "SFC_JacobianOperator.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
JacobianOperator::JacobianOperator( 
    const Teuchos::RCP<NonlinearProblem>& nonlinear_problem,
    const double epsilon )
    : d_nonlinear_problem( nonlinear_problem )
    , d_epsilon( epsilon )
{ 
    SFC_REQUIRE( Teuchos::nonull(d_nonlinear_problem) );
    SFC_REQUIRE( 0.0 < epsilon );
}

//---------------------------------------------------------------------------//
/*!
 * \brief Destructor.
 */
JacobianOperator::~JacobianOperator()
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * \brief Jacobian-free Apply operation. SFC only operates on vectors.
 */
int JacobianOperator::Apply( const Epetra_MultiVector& X, 
                             Epetra_MultiVector& Y ) const
{
    Teuchos::RCP<Epetra_Vector> x = Teuchos::rcp( X(0), false );
    Teuchos::RCP<Epetra_Vector> y = Teuchos::rcp( Y(0), false );

    Teuchos::RCP<Epetra_Vector> perturbed_F = 
	Teuchos::rcp( new Epetra_Vector(x->Map()) );

    Teuchos::RCP<Epetra_Vector> perturbed_U = 
	Teuchos::rcp( new Epetra_Vector(x->Map()) );
    int epetra_error = 0;
    epetra_error = 
	perturbed_U->Update( 1.0, *(d_nonlinear_problem->getU()), 
			     epsilon, *x, 0.0 );
    SFC_CHECK( 0 == epetra_error );

    d_nonlinear_problem->getModelEvaluator()->evaluate( perturbed_U,
							perturbed_F );

    epetra_error = y->Update( 1.0, *perturbed_F, 
			     -1.0, *(d_nonlinear_problem->getF()), 0.0 );
    SFC_CHECK( 0 == epetra_error );

    epetra_error = y->Scale( 1.0 / epsilon );
    SFC_CHECK( 0 == epetra_error );

    return 0;
}

//---------------------------------------------------------------------------//
/*!
 * \brief Get the fully formed operator to build preconditioners.
 */
Teuchos::RCP<Epetra_CrsMatrix> JacobianOperator::getCrsMatrix() const
{

}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_JacobianOperator.cpp
//---------------------------------------------------------------------------//

