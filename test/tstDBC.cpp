//---------------------------------------------------------------------------//
/*
  Copyright (c) 2012, Stuart R. Slattery
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
 * \file   tstDBC.cpp
 * \author Stuart Slattery
 * \brief  DBC unit tests.
 */
//---------------------------------------------------------------------------//

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <SFC_config.hpp>
#include <SFC_DBC.hpp>

#include "Teuchos_UnitTestHarness.hpp"
#include "Teuchos_RCP.hpp"
#include "Teuchos_Array.hpp"
#include "Teuchos_DefaultComm.hpp"
#include "Teuchos_CommHelpers.hpp"

//---------------------------------------------------------------------------//
// HELPER FUNCTIONS
//---------------------------------------------------------------------------//

// Get the default communicator.
template<class Ordinal>
Teuchos::RCP<const Teuchos::Comm<Ordinal> > getDefaultComm()
{
#ifdef HAVE_MPI
    return Teuchos::DefaultComm<Ordinal>::getComm();
#else
    return Teuchos::rcp(new Teuchos::SerialComm<Ordinal>() );
#endif
}

//---------------------------------------------------------------------------//
// Tests.
//---------------------------------------------------------------------------//
// Check that a SFC::Assertion looks different than a
// std::runtime_error as it inherits from std::logic_error.
TEUCHOS_UNIT_TEST( Assertion, differentiation_test )
{
    try
    {
	throw std::runtime_error( "runtime error" );
    }
    catch( const SFC::Assertion& assertion )
    {
	TEST_ASSERT( 0 );
    }
    catch( ... )
    {
	TEST_ASSERT( 1 );
    }
}

//---------------------------------------------------------------------------//
// Check that a SFC::Assertion can be caught and the appropriate
// error message is written.
TEUCHOS_UNIT_TEST( Assertion, message_test )
{
    std::string message;

    try
    {
	throw SFC::Assertion( "cond", "file", 12 );
    }
    catch( const SFC::Assertion& assertion )
    {
	message = std::string( assertion.what() );
    }
    catch( ... )
    {
	TEST_ASSERT( 0 );
    }

    const std::string true_message( 
	"SFC Assertion: cond, failed in file, line 12.\n" );
    TEST_ASSERT( 0 == message.compare( true_message ) );
}

//---------------------------------------------------------------------------//
// Check that we can throw a nemesis assertion with throwAssertion.
TEUCHOS_UNIT_TEST( Assertion, throw_test )
{
    try
    {
	const std::string message( "message" );
	const std::string file( "file" );
	const int line( 12 );
	SFC::throwAssertion( message, file, line );
	throw std::runtime_error( "this shouldn't be thrown" );
    }    
    catch( const SFC::Assertion& assertion )
    {
	TEST_ASSERT( 1 );	
    }
    catch( ... )
    {
	TEST_ASSERT( 0 );
    }
}

//---------------------------------------------------------------------------//
// Test the precondition check for DBC.
TEUCHOS_UNIT_TEST( Assertion, precondition_test )
{
    try 
    {
	SFC_REQUIRE( 0 );
	throw std::runtime_error( "this shouldn't be thrown" );
    }
    catch( const SFC::Assertion& assertion )
    {
#if HAVE_SFC_DBC
	std::string message( assertion.what() );
	std::string true_message( "SFC Assertion: 0, failed in" );
	std::string::size_type idx = message.find( true_message );
	if ( idx == std::string::npos )
	{
	    TEST_ASSERT( 0 );
	}
#else
	TEST_ASSERT( 0 );
#endif
    }
    catch( ... )
    {
#if HAVE_SFC_DBC
	TEST_ASSERT( 0 );
#endif
    }
}

//---------------------------------------------------------------------------//
// Test the postcondition check for DBC.
TEUCHOS_UNIT_TEST( Assertion, postcondition_test )
{
    try 
    {
	SFC_ENSURE( 0 );
	throw std::runtime_error( "this shouldn't be thrown" );
    }
    catch( const SFC::Assertion& assertion )
    {
#if HAVE_SFC_DBC
	std::string message( assertion.what() );
	std::string true_message( "SFC Assertion: 0, failed in" );
	std::string::size_type idx = message.find( true_message );
	if ( idx == std::string::npos )
	{
	    TEST_ASSERT( 0 );
	}
#else
	TEST_ASSERT( 0 );
#endif
    }
    catch( ... )
    {
#if HAVE_SFC_DBC
	TEST_ASSERT( 0 );
#endif
    }
}

//---------------------------------------------------------------------------//
// Test the invariant check for DBC.
TEUCHOS_UNIT_TEST( Assertion, invariant_test )
{
    try 
    {
	SFC_CHECK( 0 );
	throw std::runtime_error( "this shouldn't be thrown" );
    }
    catch( const SFC::Assertion& assertion )
    {
#if HAVE_SFC_DBC
	std::string message( assertion.what() );
	std::string true_message( "SFC Assertion: 0, failed in" );
	std::string::size_type idx = message.find( true_message );
	if ( idx == std::string::npos )
	{
	    TEST_ASSERT( 0 );
	}
#else
	TEST_ASSERT( 0 );
#endif
    }
    catch( ... )
    {
#if HAVE_SFC_DBC
	TEST_ASSERT( 0 );
#endif
    }
}

//---------------------------------------------------------------------------//
// Test that we can remember a value and check it with DBC.
TEUCHOS_UNIT_TEST( Assertion, remember_test )
{
    SFC_REMEMBER( int test_value_1 = 0 );
    SFC_REMEMBER( int test_value_2 = 1 );
 
    try 
    {
	SFC_CHECK( test_value_1 );
    }
    catch( const SFC::Assertion& assertion )
    {
#if HAVE_SFC_DBC
	TEST_ASSERT( 1 );
#else
	TEST_ASSERT( 0 );
#endif
    }
    catch( ... )
    {
#if HAVE_SFC_DBC
	TEST_ASSERT( 0 );
#endif
    }

    try 
    {
	SFC_CHECK( test_value_2 );
	TEST_ASSERT( 1 );
    }
    catch( ... )
    {
	TEST_ASSERT( 0 );
    }
}

//---------------------------------------------------------------------------//
// Test the insist macro for DBC.
TEUCHOS_UNIT_TEST( Assertion, _test )
{
    try 
    {
	std::string in_message( "test message content" );
	SFC_INSIST( 0, in_message );
	throw std::runtime_error( "this shouldn't be thrown" );
    }
    catch( const SFC::Assertion& assertion )
    {
	TEST_ASSERT( 1 );

	std::string message( assertion.what() );
	std::string true_message( "test message content" );
	std::string::size_type idx = message.find( true_message );
	if ( idx == std::string::npos )
	{
	    TEST_ASSERT( 0 );
	}
    }
    catch( ... )
    {
	TEST_ASSERT( 0 );
    }
}

//---------------------------------------------------------------------------//
// end tstDBC.cpp
//---------------------------------------------------------------------------//
