//---------------------------------------------------------------------------//
/*!
 * \file conduction_main.cpp
 * \author Stuart R. Slattery
 * \brief Thermal conduction driver.
 */
//---------------------------------------------------------------------------//

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <string>

#include "ConductionEvaluator.hpp"

#include <SFC_NonlinearProblem.hpp>
#include <SFC_ModelEvaluator.hpp>
#include <SFC_NewtonSolver.hpp>

#include <Teuchos_RCP.hpp>
#include <Teuchos_ParameterList.hpp>
#include <Teuchos_CommandLineProcessor.hpp>
#include <Teuchos_XMLParameterListCoreHelpers.hpp>
#include <Teuchos_ArrayView.hpp>

#include <Epetra_Comm.h>
#include <Epetra_Vector.h>
#include <Epetra_Map.h>

#ifdef HAVE_MPI
#include <mpi.h>
#include <Epetra_MpiComm.h>
#else
#include <Epetra_SerialComm.h>
#endif

//---------------------------------------------------------------------------//
// Main function driver for the coupled Wave/Damper problem.
int main(int argc, char* argv[])
{
    // Parallel setup.
#ifdef HAVE_MPI
    MPI_Init( &argc, &argv );
    Epetra_MpiComm comm( MPI_COMM_WORLD );
#else
    Epetra_SerialComm Comm;
#endif

    // Read in command line options.
    std::string xml_input_filename;
    Teuchos::CommandLineProcessor clp(false);
    clp.setOption( "i",
		   &xml_input_filename,
		   "The XML file to read into a parameter list" );
    clp.parse(argc,argv);

    // Build the parameter list from the xml input.
    Teuchos::RCP<Teuchos::ParameterList> plist =
	Teuchos::rcp( new Teuchos::ParameterList() );
    Teuchos::updateParametersFromXmlFile(
	xml_input_filename, Teuchos::inoutArg(*plist) );

    // Epetra Setup.
    int problem_size = plist->get<int>( "Problem Size" );
    Epetra_Map map( problem_size, 0, comm );
    Teuchos::RCP<Epetra_Vector> u = Teuchos::rcp( new Epetra_Vector(map) );
    u->PutScalar( 200.0 );
    
    // Boundary conditions.
    (*u)[0] = plist->get<double>( "Left BC" );
    (*u)[problem_size-1] = plist->get<double>( "Right BC" );

    // Model Setup.
    Teuchos::RCP<SFC::ModelEvaluator> model_evaluator =
	Teuchos::rcp( new Conduction::ConductionEvaluator() );

    // Thermal source.
    double q_val = plist->get<double>( "Thermal Source" );
    Teuchos::RCP<Epetra_Vector> q = Teuchos::rcp( new Epetra_Vector(map) );
    q->PutScalar( q_val );
    Teuchos::rcp_dynamic_cast<Conduction::ConductionEvaluator>(
        model_evaluator )->setQ( q );

    // Nonlinear Problem Setup.
    Teuchos::RCP<SFC::NonlinearProblem> nonlinear_problem = Teuchos::rcp( 
        new SFC::NonlinearProblem(model_evaluator, u) );

    // Build the Newton solver and solve the nonlinear problem.
    SFC::NewtonSolver nonlinear_solver( nonlinear_problem, plist );
    nonlinear_solver.solve();

    // Output the results to a file.
    std::ofstream ofile;
    ofile.open( "conduction_example.dat" );
    for ( int i = 0; i < problem_size; ++i )
    {
	ofile << std::setprecision(10) << (*u)[i] << std::endl;
    }
    ofile.close();
}


//---------------------------------------------------------------------------//
// end cxx_main.cpp
//---------------------------------------------------------------------------//

