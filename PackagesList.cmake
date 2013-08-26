#
# See documentation in Trilinos preCopyrightTrilinos/ExtraExternalRepositories.cmake
#

INCLUDE(TribitsListHelpers)

SET( SFC_PACKAGES_AND_DIRS_AND_CLASSIFICATIONS
  SFC         .     SS
  )

PACKAGE_DISABLE_ON_PLATFORMS(SFC Windows)
