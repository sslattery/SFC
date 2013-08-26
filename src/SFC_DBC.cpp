//---------------------------------------------------------------------------//
/*!
 * \file   SFC_Assertion.cpp
 * \author Stuart Slattery
 * \brief  Assertions for error handling and Design-by-Contract.
 */
//---------------------------------------------------------------------------//

#include <sstream>

#include "SFC_DBC.hpp"

namespace SFC
{
//---------------------------------------------------------------------------//
// Assertion functions.
//---------------------------------------------------------------------------//
/*!
 * \brief Build an assertion output from advanced constructor arguments.
 *
 * \param cond A string containing the assertion condition that failed.
 *
 * \param field A string containing the file name in which the assertion
 * failed. 
 *
 * \param line The line number at which the assertion failed.
 *
 * \return Assertion output.
 */
std::string Assertion::generate_output( 
    const std::string& cond, const std::string& file, const int line ) const
{
    std::ostringstream output;
    output << "SFC Assertion: " << cond << ", failed in " << file
	   << ", line " << line  << "." << std::endl;
    return output.str();
}

//---------------------------------------------------------------------------//
// Throw functions.
//---------------------------------------------------------------------------//
/*!
 * \brief Throw a SFC::Assertion.
 *
 * \param cond A string containing the assertion condition that failed.
 *
 * \param field A string containing the file name in which the assertion
 * failed. 
 *
 * \param line The line number at which the assertion failed.
 */
void throwAssertion( const std::string& cond, const std::string& file,
		     const int line )
{
    throw Assertion( cond, file, line );
}

//---------------------------------------------------------------------------//
/*!
 * \brief Insist a statement is true with a provided message.
 *
 * \param cond A string containing the assertion condition that failed.
 *
 * \param msg A message to provide if the assertion is thrown.
 * \param field A string containing the file name in which the assertion
 * failed. 
 *
 * \param line The line number at which the assertion failed.
 */
void insist( const std::string& cond, const std::string& msg,
	     const std::string& file, const int line )
{
    std::ostringstream output_msg;
    output_msg <<  "Insist: " << cond << ", failed in "
	      << file << ":" << line << std::endl
	      << "The following message was provided:" << std::endl
	      << "\"" << msg << "\"" << std::endl;
    throw Assertion( output_msg.str() );
}


//---------------------------------------------------------------------------//

} // end namespace SFC

//---------------------------------------------------------------------------//
// end SFC_Assertion.cpp
//---------------------------------------------------------------------------//
