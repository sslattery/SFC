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
 * \file   SFC_JacobianOperator.hpp
 * \author Stuart Slattery
 * \brief  Jacobian operator class.
 */
//---------------------------------------------------------------------------//

#ifndef SFC_JACOBIANOPERATOR_HPP
#define SFC_JACOBIANOPERATOR_HPP

#include "SFC_NonlinearResidual.hpp"

#include <Teuchos_RCP.hpp>

#include <Epetra_Vector.h>
#include <Epetra_Operator.h>
#include <Epetra_CrsMatrix.h>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Jacobian operator.
 */
//---------------------------------------------------------------------------//
class JacobianOperator : public Epetra_Operator
{
  public:

    // Constructor.
    JacobianOperator( const Teuchos::RCP<NonlinearResidual>& F,
                      const Teuchos::RCP<Epetra_Vector>& u,
                      const double epsilon );

    // Destructor.
    ~JacobianOperator();

    //@{
    //! Epetra_Operator interface.
    // Set to false for not supporting the transpose.
    int SetUseTranspose(bool UseTranspose) { return 0; }

    // Jacobian-free Apply operation.
    int Apply( const Epetra_MultiVector& X, Epetra_MultiVector& Y ) const;

    // Inverse apply operation.
    int ApplyInverse( const Epetra_MultiVector& X, Epetra_MultiVector& Y ) const;
    { return 0; }

    // Get the infinity norm.
    double NormInf() const { return 0.0; }

    // Returns a character string describing the operator
    const char * Label() const { return std::string("SFC Jacobian").c_str(); }

    // Returns the current UseTranspose setting.
    bool UseTranspose() const { return false; }

    // Returns true if the \e this object can provide an approximate Inf-norm,
    // false otherwise.
    bool HasNormInf() const { return false; }

    // Returns a pointer to the Epetra_Comm communicator associated with this
    // operator.
    const Epetra_Comm & Comm() const { return d_u->Comm(); }

    // Returns the Epetra_Map object associated with the domain of this
    // operator.
    const Epetra_Map & OperatorDomainMap() const { return d_u->Map(); }

    // Returns the Epetra_Map object associated with the range of this
    // operator.
    const Epetra_Map & OperatorRangeMap() const { return d_u->Map(); }
    //@}

    // Get the fully formed operator to build preconditioners.
    Teuchos::RCP<Epetra_CrsMatrix> getCrsMatrix() const;

  private:
    
    // Nonlinear residual.
    Teuchos::RCP<NonlinearResidual> d_F;

    // Nonlinear solution.
    Teuchos::RCP<Epetra_Vector> d_u;

    // Jacobian-free perturbation parameter.
    double d_epsilon;
};

#endif // end SFC_JACOBIANOPERATOR_HPP

//---------------------------------------------------------------------------//
// end SFC_JacobianOperator.hpp
//---------------------------------------------------------------------------//

