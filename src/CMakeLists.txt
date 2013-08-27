INCLUDE(TribitsLibraryMacros)

#
# A) Package-specific configuration options
#

TRIBITS_CONFIGURE_FILE(${PACKAGE_NAME}_config.hpp)

#
# B) Define the header and source files (and directories)
#

SET(HEADERS "")
SET(SOURCES "")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

SET(HEADERS ${HEADERS}
  ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}_config.hpp
  )

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

APPEND_SET(HEADERS
  SFC_AveragePerturbation.hpp
  SFC_BasicPerturbation.hpp
  SFC_DBC.hpp
  SFC_JacobianOperator.hpp
  SFC_ModelEvaluator.hpp
  SFC_NonlinearProblem.hpp
  SFC_PerturbationParameter.hpp
  ) 

APPEND_SET(SOURCES
  SFC_AveragePerturbation.cpp
  SFC_BasicPerturbation.cpp
  SFC_DBC.cpp
  SFC_JacobianOperator.cpp
  SFC_NonlinearProblem.cpp
  )

#
# C) Define the targets for package's library(s)
#

TRIBITS_ADD_LIBRARY(
  sfc	
  HEADERS ${HEADERS}
  SOURCES ${SOURCES}
  )

INSTALL(FILES 
  ${HEADERS} 
  DESTINATION 
  ${CMAKE_INSTALL_PREFIX}/include/
)
