//---------------------------------------------------------------------------//
/*!
 * \file cxx_main.cpp
 * \author Stuart R. Slattery
 * \brief SimpleExample Driver.
 */
//---------------------------------------------------------------------------//

#include <iostream>
#include <string>

#include "SimpleEvaluator.hpp"

#include <SFC_NonlinearProblem.hpp>
#include <SFC_ModelEvaluator.hpp>
#include <SFC_NewtonSolver.hpp>

#include <Teuchos_RCP.hpp>
#include <Teuchos_ParameterList.hpp>
#include <Teuchos_ArrayView.hpp>

#include <Epetra_SerialComm.h>
#include <Epetra_Vector.h>
#include <Epetra_Map.h>

//---------------------------------------------------------------------------//
// Main function driver for the coupled Wave/Damper problem.
int main(int argc, char* argv[])
{
    // Epetra Setup.
    Epetra_SerialComm comm;
    int problem_size = 1;
    Epetra_Map map( problem_size, 0, comm );
    Teuchos::RCP<Epetra_Vector> u = Teuchos::rcp( new Epetra_Vector(map) );

    // Model Setup.
    double a = 1.0;
    double b = 1.0;
    double c = 0.0;
    Teuchos::RCP<SFC::ModelEvaluator> model_evaluator =
        Teuchos::rcp( new SimpleExample::SimpleEvaluator(a,b,c) );

    // Nonlinear Problem Setup.
    Teuchos::RCP<SFC::NonlinearProblem> nonlinear_problem = Teuchos::rcp( 
        new SFC::NonlinearProblem(model_evaluator, u) );

    // Nonlinear Solver Parameters
    Teuchos::RCP<Teuchos::ParameterList> parameters = Teuchos::parameterList();
    parameters->set<int>( "Newton Maximum Iterations", 100 );
    parameters->set<double>( "Newton Convergence Tolerance", 100 );

    // Linear solver parameters
    parameters->set<int>( "GMRES Maximum Iterations", 100 );

    // Forcing term parameters
    parameters->set<std::string>( "Forcing Term Type", "Constant" );
    parameters->set<double>( "Constant Forcing Term", 1.0e-4 );

    // Globalization parameters
    parameters->set<std::string>( "Globalization Type", "None" );

    // Perturbation parameters
    parameters->set<std::string>( "Perturbation Type", "Basic" );

    // Build the Newton solver and solve the nonlinear problem.
    SFC::NewtonSolver nonlinear_solver( nonlinear_problem, parameters );
    nonlinear_solver.solve();

    // Output the results.
    double* u_ptr;
    u->ExtractView( &u_ptr );
    std::cout << Teuchos::ArrayView<double>( u_ptr, problem_size ) << std::endl;
}


//---------------------------------------------------------------------------//
// end cxx_main.cpp
//---------------------------------------------------------------------------//

