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
 * \file   SFC_NewtonSolver.cpp
 * \author Stuart Slattery
 * \brief  Newton's method.
 */
//---------------------------------------------------------------------------//

#include <iostream>

#include "SFC_DBC.hpp"
#include "SFC_NewtonSolver.hpp"
#include "SFC_JacobianOperator.hpp"
#include "SFC_PerturbationParameterFactory.hpp"
#include "SFC_GlobalizationFactory.hpp"
#include "SFC_ForcingTermFactory.hpp"

#include <Epetra_Vector.h>
#include <Epetra_LinearProblem.h>
#include <Epetra_Operator.h>

#include <AztecOO.h>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
NewtonSolver::NewtonSolver( 
    const Teuchos::RCP<NonlinearProblem>& nonlinear_problem,
    const Teuchos::RCP<Teuchos::ParameterList>& parameters )
    : d_nonlinear_problem( nonlinear_problem )
    , d_parameters( parameters )
{
    SFC_REQUIRE( Teuchos::nonnull(d_nonlinear_problem) );
    SFC_REQUIRE( Teuchos::nonnull(d_parameters) );
}

//---------------------------------------------------------------------------//
/*!
 * \brief Solve the nonlinear problem
 */
void NewtonSolver::solve()
{
    // Setup the globalization method.
    GlobalizationFactory globalization_factory;
    Teuchos::RCP<Globalization> globalization = 
        globalization_factory.create( *d_parameters );
    globalization->setNonlinearProblem( d_nonlinear_problem );
    Teuchos::RCP<Epetra_Vector> global_update = Teuchos::rcp(
        new Epetra_Vector(d_nonlinear_problem->getU()->Map()) );

    // Build the forcing term.
    ForcingTermFactory forcing_term_factory;
    Teuchos::RCP<ForcingTerm> forcing_term = 
        forcing_term_factory.create( *d_parameters );
    forcing_term->setNonlinearProblem( d_nonlinear_problem );

    // Build the matrix-free perturbation method.
    PerturbationParameterFactory perturbation_factory;
    Teuchos::RCP<PerturbationParameter> perturbation =
        perturbation_factory.create( *d_parameters );

    // Build the Jacobian.
    Teuchos::RCP<JacobianOperator> jacobian = Teuchos::rcp(
        new JacobianOperator(d_nonlinear_problem, perturbation) );

    // Build the linear model.
    Teuchos::RCP<Epetra_Operator> epetra_jacobian = jacobian;
    Teuchos::RCP<Epetra_Vector> newton_update = Teuchos::rcp(
        new Epetra_Vector(d_nonlinear_problem->getU()->Map()) );
    Teuchos::RCP<Epetra_Vector> newton_rhs = Teuchos::rcp(
        new Epetra_Vector(d_nonlinear_problem->getU()->Map()) );
    Epetra_LinearProblem linear_problem( epetra_jacobian.getRawPtr(),
                                         newton_update.getRawPtr(),
                                         newton_rhs.getRawPtr() );

    // Build the GMRES solver.
    int aztec_error = 0;
    AztecOO gmres_solver( linear_problem );
    aztec_error = gmres_solver.SetAztecOption( AZ_solver, AZ_gmres );
    SFC_CHECK( 0 == aztec_error );
    int max_gmres_iters = d_parameters->get<int>( "GMRES Maximum Iterations" );

    // Get Newton solver parameters.
    int max_newton_iters = d_parameters->get<int>( "Newton Maximum Iterations" );
    double newton_tolerance = 
        d_parameters->get<int>( "Newton Convergence Tolerance" );

    // Evaluate the initial nonlinear residual and get the preliminary norms.
    int epetra_error = 0.0;
    d_nonlinear_problem->evaluate();
    double norm_F_0 = 0.0;
    epetra_error = d_nonlinear_problem->getF()->Norm2( &norm_F_0 );
    SFC_CHECK( 0 == epetra_error );
    double norm_F_k = 0.0;
    epetra_error = d_nonlinear_problem->getF()->Norm2( &norm_F_k );
    SFC_CHECK( 0 == epetra_error );

    // Perform Newton iterations until convergence.
    bool converged = ( (norm_F_k / norm_F_0) < newton_tolerance );
    int num_iters = 0;
    double eta = 0.0;
    while ( !converged && num_iters < max_newton_iters )
    {
        // Setup the linear model.
        epetra_error = 
            newton_rhs->Update( -1.0, *(d_nonlinear_problem->getF()), 0.0 );
        SFC_CHECK( 0 == epetra_error );

        // Solve the linear model.
        eta = forcing_term->calculateForcingTerm();
        aztec_error = gmres_solver.Iterate( max_gmres_iters, eta );
        SFC_CHECK( 0 == aztec_error );

        // Apply globalization to the Newton Vector.
        globalization->calculateUpdate( newton_update, global_update );

        // Update the nonlinear solution.
        epetra_error = 
            d_nonlinear_problem->getU()->Update( 1.0, *global_update, 1.0 );
        SFC_CHECK( 0 == epetra_error );

        // Check for convergence of the nonlinear residual.
        converged = ( (norm_F_k / norm_F_0) < newton_tolerance );
        ++num_iters;

        // Output.
        std::cout << "Newton Iteration " << num_iters
                  << ": ||F||^k / ||F||^0 = " << norm_F_k / norm_F_0 
                  << std::endl;
    }
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_NewtonSolver.cpp
//---------------------------------------------------------------------------//

