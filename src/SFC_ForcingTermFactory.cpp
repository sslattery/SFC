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
 * \file   SFC_ForcingTermFactory.cpp
 * \author Stuart Slattery
 * \brief  Factory for Jacobian-free perturbation parameters.
 */
//---------------------------------------------------------------------------//

#include "SFC_DBC.hpp"
#include "SFC_ForcingTermFactory.hpp"
#include "SFC_ConstantForcingTerm.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
ForcingTermFactory::ForcingTermFactory()
{
    d_name_map["Constant"] = CONSTANT;
}

//---------------------------------------------------------------------------//
/*!
 * \brief Creation method.
 */
Teuchos::RCP<ForcingTerm> 
ForcingTermFactory::create( const Teuchos::ParameterList& parameters )
{
    std::string name = "Constant";
    if ( parameters.isParameter("Forcing Term Type") )
    {
        name = parameters.get( "Forcing Term Type" );
    }

    Teuchos::RCP<ForcingTerm> forcing_term;

    std::map<std::string,int>::const_iterator id = d_name_map.find( name );
    SFC_CHECK( id != d_name_map.end() );

    switch( id->second )
    {
        case CONSTANT
            forcing_term = Teuchos::rcp( new ConstantForcingTerm() );
            break

        default:
            forcing_term = Teuchos::rcp( new ConstantForcingTerm() );
            break;
    }

    SFC_ENSURE( Teuchos::nonnull(forcing_term) );
    return forcing_term;
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_ForcingTermFactory.hpp
//---------------------------------------------------------------------------//

