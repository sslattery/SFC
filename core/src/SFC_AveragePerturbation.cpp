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
 * \file   SFC_AveragePerturbation.cpp
 * \author Stuart Slattery
 * \brief  Average Jacobian-free perturbation parameter.
 */
//---------------------------------------------------------------------------//

#include <limits>
#include <cmath>

#include "SFC_DBC.hpp"
#include "SFC_AveragePerturbation.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Given a the nonlinear solution and the vector on which the Jacobian
 * is acting, generate a perturbation parameter for the Jacobian-free
 * approximation. Eq(11) in Knoll and Keyes 2004 JFNK survey paper.
 */
double AveragePerturbation::calculatePerturbation( 
    const Teuchos::RCP<const Epetra_Vector>& u,
    const Teuchos::RCP<const Epetra_Vector>& v )
{
    int epetra_error = 0;
    double b = 10000 * std::numeric_limits<double>::epsilon();
    int n = u->GlobalLength();
    double v_norm = 0.0;
    epetra_error = v->Norm2( &v_norm );
    SFC_CHECK( 0 == epetra_error );
    double u_norm = 0.0;
    epetra_error = u->Norm1( &u_norm );
    SFC_CHECK( 0 == epetra_error );
    return ( b * u_norm ) / ( n * v_norm ) + b;
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_AveragePerturbation.cpp
//---------------------------------------------------------------------------//

