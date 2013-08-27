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
 * \file   SimpleEvaluator.hpp
 * \author Stuart Slattery
 * \brief  Simple model evaluator for example problem.
 */
//---------------------------------------------------------------------------//

#ifndef SIMPLEEVALUATOR_HPP
#define SIMPLEEVALUATOR_HPP

#include <SFC_ModelEvaluator.hpp>

#include <Teuchos_RCP.hpp>

#include <Epetra_Vector.h>

namespace SimpleExample
{
//---------------------------------------------------------------------------//
/*!
 * \brief Simple 1D parabolic equation model evaluator: 
 *
 *                        a * (du^2)(d^2x) + b * u - c = 0
 */
//---------------------------------------------------------------------------//
class SimpleEvaluator : public SFC::ModelEvaluator
{
  public:

    // Constructor.
    SimpleEvaluator( const double a, const double b, const double c );

    //! Destructor.
    ~SimpleEvaluator()
    { /* ... */ }

    // Given a vector, evaluate the model and generate a nonlinear residual.
    void evaluate( const Teuchos::RCP<Epetra_Vector>& u,
                   Teuchos::RCP<Epetra_Vector>& F );

  private:

    double d_a;
    double d_b;
    double d_c;
};

//---------------------------------------------------------------------------//

} // end namespace SimpleExample

#endif // end SIMPLEEVALUATOR_HPP

//---------------------------------------------------------------------------//
// end SimpleEvaluator.hpp
//---------------------------------------------------------------------------//

