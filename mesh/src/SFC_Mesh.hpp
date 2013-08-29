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
 * \file   SFC_Mesh.hpp
 * \author Stuart Slattery
 * \brief  Interface definition for mesh.
 */
//---------------------------------------------------------------------------//

#ifndef SFC_MESH_HPP
#define SFC_MESH_HPP

#include <Teuchos_RCP.hpp>
#include <Teuchos_ArrayRCP.hpp>

#include <Epetra_BlockMap.h>
#include <Epetra_MultiVector.h>

#include <Shards_CellTopology.hpp>

namespace SFC
{
//---------------------------------------------------------------------------//
/*!
 * \brief Base class for mesh.
 */
//---------------------------------------------------------------------------//
class Mesh
{
  public:

    //! Constructor.
    Mesh()
    { /* ... */ }

    //! Destructor.
    virtual ~Mesh()
    { /* ... */ }

    //! Set the topology of the mesh.
    virtual shards::CellTopology topology() const = 0;

    //! Get the number of local vertices in the mesh.
    virtual int numLocalVertices() const = 0;

    //! Get the number of global vertices in the mesh.
    virtual int numGlobalVertices() const = 0;

    //! Get the number of local elements in the mesh.
    virtual int numLocalElements() const = 0;

    //! Get the number of global elements in the mesh.
    virtual int numGlobalElements() const = 0;

    //! Get the vertex id map.
    virtual Teuchos::RCP<Epetra_BlockMap> getVertexMap() const = 0;

    //! Get the element id map.
    virtual Teuchos::RCP<Epetra_BlockMap> getElementMap() const = 0;

    //! Get the vertices that construct an element.
    virtual void getElementAdjacentVertices( 
	const int element_id, Teuchos::ArrayRCP<int> vertex_ids ) const = 0;

    //! Get the elements that are adjacent to an element.
    virtual void getElementAdjacentElements(
	const int element_id, Teuchos::ArrayRCP<int> element_ids ) const = 0;

    //! Get the coordinates of the vertices.
    virtual Teuchos::RCP<Epetra_Multivector> getVertexCoords() const = 0;
};

//---------------------------------------------------------------------------//

} // end namespace SFC

#endif // end SFC_MESH_HPP

//---------------------------------------------------------------------------//
// end SFC_Mesh.hpp
//---------------------------------------------------------------------------//

