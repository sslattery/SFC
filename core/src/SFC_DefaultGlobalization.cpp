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
 * \file   SFC_DefaultGlobalization.cpp
 * \author Stuart Slattery
 * \brief  Default globalization.
 */
//---------------------------------------------------------------------------//

#include "SFC_DBC.hpp"
#include "SFC_DefaultGlobalization.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
DefaultGlobalization::DefaultGlobalization()
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * \brief Set the nonlinear problem
 */
void DefaultGlobalization::setNonlinearProblem( 
    const Teuchos::RCP<NonlinearProblem>& nonlinear_problem )
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * \brief Given a Newton update simply return it as this does no
*globalization. 
*/
void DefaultGlobalization::calculateUpdate( 
    const Teuchos::RCP<const Epetra_Vector>& newton_update,
    Teuchos::RCP<Epetra_Vector>& global_update )
{
    global_update = Teuchos::rcp_const_cast<Epetra_Vector>(newton_update);
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_DefaultGlobalization.cpp
//---------------------------------------------------------------------------//

