##---------------------------------------------------------------------------##
## SFC/CMakeLists.txt
## Stuart R. Slattery
##---------------------------------------------------------------------------##
## TriBITS package file for the SFC package
##---------------------------------------------------------------------------##

INCLUDE(TribitsPackageMacros)
INCLUDE(TribitsAddOptionAndDefine)

##---------------------------------------------------------------------------##
## A) Define the package
##---------------------------------------------------------------------------##

TRIBITS_PACKAGE(SFC)

##---------------------------------------------------------------------------##
## B) Set up package-specific options
##---------------------------------------------------------------------------##

# Design-by-Contract
TRIBITS_ADD_OPTION_AND_DEFINE(
	${PACKAGE_NAME}_ENABLE_DBC
	HAVE_SFC_DBC
	"Enable Design-by-Contract checks. WARNING: use for debug checking but disable for production runs as it incurs a significant overhead."
	${${PROJECT_NAME}_ENABLE_DEBUG}
)

TRIBITS_ADD_DEBUG_OPTION()

TRIBITS_ADD_SHOW_DEPRECATED_WARNINGS_OPTION()

##---------------------------------------------------------------------------##
## C) Add library, test, and examples.
##---------------------------------------------------------------------------##

ADD_SUBDIRECTORY(src)

TRIBITS_ADD_TEST_DIRECTORIES(test)

TRIBITS_ADD_EXAMPLE_DIRECTORIES(example)

##---------------------------------------------------------------------------##
## D) Do standard postprocessing
##---------------------------------------------------------------------------##

TRIBITS_PACKAGE_POSTPROCESS()

##---------------------------------------------------------------------------##
##                      end of SFC/CMakeLists.txt
##---------------------------------------------------------------------------##
