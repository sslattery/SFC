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
 * \file   SFC_PerturbationParameterFactory.cpp
 * \author Stuart Slattery
 * \brief  Factory for Jacobian-free perturbation parameters.
 */
//---------------------------------------------------------------------------//

#include "SFC_DBC.hpp"
#include "SFC_PerturbationParameterFactory.hpp"
#include "SFC_BasicPerturbation.hpp"
#include "SFC_AveragePerturbation.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Constructor.
 */
PerturbationParameterFactory::PerturbationParameterFactory()
{
    d_name_map["Basic"] = BASIC;
    d_name_map["Average"] = AVERAGE;
}

//---------------------------------------------------------------------------//
/*!
 * \brief Creation method.
 */
Teuchos::RCP<PerturbationParameter> 
PerturbationParameterFactory::create( const Teuchos::ParameterList& parameters )
{
    std::string name = parameters.get<std::string>( "Perturbation Type" );

    Teuchos::RCP<PerturbationParameter> perturbation;

    std::map<std::string,int>::const_iterator id = d_name_map.find( name );
    SFC_CHECK( id != d_name_map.end() );

    switch( id->second )
    {
        case BASIC:
            perturbation = Teuchos::rcp( new BasicPerturbation() );
            break;

        case AVERAGE:
            perturbation = Teuchos::rcp( new AveragePerturbation() );
            break;

        default:
            throw Assertion( "Pertubation type not supported!" );
            break;
    }

    SFC_ENSURE( Teuchos::nonnull(perturbation) );
    return perturbation;
}

//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_PerturbationParameterFactory.hpp
//---------------------------------------------------------------------------//

