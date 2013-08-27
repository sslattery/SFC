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
 * \file   SFC_NonlinearProblem.hpp
 * \author Stuart Slattery
 * \brief  Nonlinear problem container.
 */
//---------------------------------------------------------------------------//

#ifndef SFC_NONLINEARPROBLEM_HPP
#define SFC_NONLINEARPROBLEM_HPP

#include "SFC_ModelEvaluator.hpp"

#include <Teuchos_RCP.hpp>

#include <Epetra_Vector.h>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Nonlinear problem container.
 */
//---------------------------------------------------------------------------//
class NonlinearProblem
{
  public:

    // Constructor.
    NonlinearProblem( const Teuchos::RCP<ModelEvaluator>& me, 
		      const Teuchos::RCP<Epetra_Vector>& u );

    // Destructor.
    ~NonlinearProblem();

    // Get the model evaluator.
    Teuchos::RCP<ModelEvaluator> getModelEvaluator() const
    { return d_me; }

    // Get the nonlinear solution.
    Teuchos::RCP<Epetra_Vector> getU() const
    { return d_u; }

    // Get the nonlinear residual.
    Teuchos::RCP<Epetra_Vector> getF() const
    { return d_F; }

    // Evaluate the nonlinear model with the current solution to update the
    // nonlinear residual.
    void evaluate();

  private:
    
    // Nonlinear model evaluator.
    Teuchos::RCP<ModelEvaluator> d_me;

    // Nonlinear residual.
    Teuchos::RCP<Epetra_Vector> d_F;

    // Nonlinear solution.
    Teuchos::RCP<Epetra_Vector> d_u;
};

} // end namespace SFC

#endif // end SFC_NONLINEARPROBLEM_HPP

//---------------------------------------------------------------------------//
// end SFC_NonlinearProblem.hpp
//---------------------------------------------------------------------------//

