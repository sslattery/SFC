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

#include "SFC_DBC.hpp"
#include "SFC_JacobianOperator.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * Constructor.
 */
JacobianOperator::JacobianOperator( const Teuchos::RCP<NonlinearResidual>& F,
                                    const Teuchos::RCP<Epetra_Vector>& u,
                                    const double epsilon )
    : d_F( F )
    , d_u( u )
    , d_epsilon( epsilon )
{ 
    SFC_REQUIRE( Teuchos::nonull(d_F) );
    SFC_REQUIRE( Teuchos::nonull(d_u) );
    SFC_REQUIRE( 0.0 < epsilon );
}

//---------------------------------------------------------------------------//
/*!
 * Destructor.
 */
JacobianOperator::~JacobianOperator()
{ /* ... */ }

//---------------------------------------------------------------------------//
/*!
 * Jacobian-free Apply operation.
 */
int JacobianOperator::Apply( const Epetra_MultiVector& X, 
                             Epetra_MultiVector& Y ) const
{
    
}

//---------------------------------------------------------------------------//
/*!
 * Get the fully formed operator to build preconditioners.
 */
JacobianOperator::Teuchos::RCP<Epetra_CrsMatrix> getCrsMatrix() const
{

}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_JacobianOperator.cpp
//---------------------------------------------------------------------------//

