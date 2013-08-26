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
 * \file   SFC_NonlinearResidual.hpp
 * \author Stuart Slattery
 * \brief  Interface definition for nonlinear residuals.
 */
//---------------------------------------------------------------------------//

#ifndef SFC_NONLINEARRESIDUAL_HPP
#define SFC_NONLINEARRESIDUAL_HPP

#include <Teuchos_RCP.hpp>

#include <Epetra_Vector.h>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Base class for nonlinear resiudals.
 */
//---------------------------------------------------------------------------//
class NonlinearResidual
{
  public:

    //! Constructor.
    NonlinearResidual()
    { /* ... */ }

    //! Destructor.
    virtual ~NonlinearResidual()
    { /* ... */ }

    //! Given a vector, evaluate the nonlinear residual.
    virtual void evaluate( const Teuchos::RCP<Epetra_Vector>& u,
                           Teuchos::RCP<Epetra_Vector>& F ) = 0;
};

#endif // end SFC_NONLINEARRESIDUAL_HPP

//---------------------------------------------------------------------------//
// end SFC_NonlinearResidual.hpp
//---------------------------------------------------------------------------//

