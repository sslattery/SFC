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

#include <algorithm>

#include "SFC_DBC.hpp"
#include "SFC_JacobianOperator.hpp"

#include <Teuchos_as.hpp>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
JacobianOperator::JacobianOperator( 
    const Teuchos::RCP<NonlinearProblem>& nonlinear_problem,
    const Teuchos::RCP<PerturbationParameter>& perturbation )
    : d_nonlinear_problem( nonlinear_problem )
    , d_perturbation( perturbation )
{
    SFC_REQUIRE( Teuchos::nonnull(d_nonlinear_problem) );
    SFC_REQUIRE( Teuchos::nonnull(d_perturbation) );

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
JacobianOperator::~JacobianOperator()
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * \brief Jacobian-free Apply operation.
 */
int JacobianOperator::Apply( const Epetra_MultiVector& X, 
                             Epetra_MultiVector& Y ) const
{
    Teuchos::RCP<const Epetra_Vector> x = Teuchos::rcp( X(0), false );
    Teuchos::RCP<Epetra_Vector> y = Teuchos::rcp( Y(0), false );

    double epsilon = d_perturbation->calculatePerturbation( 
        d_nonlinear_problem->getU(), x );

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
    int epetra_error = 0;
    int global_length = d_nonlinear_problem->getU()->GlobalLength();
    int local_length =  d_nonlinear_problem->getU()->MyLength();
    Epetra_Vector column_basis( d_nonlinear_problem->getU()->Map() );
    Epetra_Vector extract_column( d_nonlinear_problem->getU()->Map() );
    Teuchos::RCP<Epetra_CrsMatrix> jacobian = 
        Teuchos::rcp( new Epetra_CrsMatrix(Copy,*d_domain_map,0) );

    for ( int n = 0; n < global_length; ++n )
    {
        epetra_error = column_basis.PutScalar( 0.0 );
        SFC_CHECK( 0 == epetra_error );
        column_basis[n] = 1.0;

        epetra_error = Apply( column_basis, extract_column );
        SFC_CHECK( 0 == epetra_error );

        for ( int i = 0; i < local_length; ++i )
        {
            if ( extract_column[i] != 0.0 )
            {
                epetra_error = 
                    jacobian->InsertGlobalValues( 
                        d_nonlinear_problem->getU()->Map().GID(i), 1, 
                        &extract_column[i], &n );
                SFC_CHECK( 0 == epetra_error );
            }
        }
    }

    jacobian->FillComplete();

    return jacobian;
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_JacobianOperator.cpp
//---------------------------------------------------------------------------//

