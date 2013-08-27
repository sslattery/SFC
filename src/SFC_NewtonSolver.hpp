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
 * \file   SFC_NewtonSolver.hpp
 * \author Stuart Slattery
 * \brief  Newton's method.
 */
//---------------------------------------------------------------------------//

#ifndef SFC_NEWTONSOLVER_HPP
#define SFC_NEWTONSOLVER_HPP

#include "SFC_NonlinearProblem.hpp"
#include "SFC_Globalization.hpp"

#include <Teuchos_RCP.hpp>
#include <Teuchos_ParameterList.hpp>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Newton's method.
 */
//---------------------------------------------------------------------------//
class NewtonSolver
{
  public:

    // Constructor.
    NewtonSolver( const Teuchos::RCP<NonlinearProblem>& nonlinear_problem,
                  const Teuchos::RCP<Teuchos::ParameterList>& parameters );

    // Destructor.
    ~NewtonSolver()
    { /* ... */ }

    // Solve the nonlinear problem
    void solve();

  private:
    
    // Nonlinear problem.
    Teuchos::RCP<NonlinearProblem> d_nonlinear_problem;

    // Globalization.
    Teuchos::RCP<Globalization> d_globalization;

    // Parameters.
    Teuchos::RCP<Teuchos::ParameterList> d_parameters;
};

//---------------------------------------------------------------------------//

} // end namespace SFC

#endif // end SFC_NEWTONSOLVER_HPP

//---------------------------------------------------------------------------//
// end SFC_NewtonSolver.hpp
//---------------------------------------------------------------------------//

