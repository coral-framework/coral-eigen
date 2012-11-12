################################################################################
# Eigen - Graphics Framework
################################################################################

CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

PROJECT( CORAL_EIGEN )

################################################################################
# Setup Coral
################################################################################

# Load Coral's CMake package
if( NOT CORAL_ROOT )
	file( TO_CMAKE_PATH "$ENV{CORAL_ROOT}" CORAL_ROOT )
endif()
set( CMAKE_MODULE_PATH "${CORAL_ROOT}/cmake" ${CMAKE_MODULE_PATH} )
find_package( Coral REQUIRED )

SET( CORAL_PATH
	"${CMAKE_BINARY_DIR}/modules"
	"${CMAKE_SOURCE_DIR}/modules"
	${CORAL_PATH}
)

################################################################################
# Installation
################################################################################

#install eigen headers
INSTALL( DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include )

# install CSL files
INSTALL( DIRECTORY ${CMAKE_SOURCE_DIR}/modules/ DESTINATION modules )

################################################################################
# Packaging
################################################################################

SET( CPACK_PACKAGE_NAME					"coral-eigen" )
SET( CPACK_PACKAGE_VERSION_MAJOR		"0" )
SET( CPACK_PACKAGE_VERSION_MINOR		"1" )
SET( CPACK_PACKAGE_VERSION_PATCH		"0" )
SET( CPACK_PACKAGE_DESCRIPTION_SUMMARY	"A module for integrating [Eigen](http://eigen.tuxfamily.org/) into Coral" )

INCLUDE( CPack )

################################################################################
# Subdirectories
################################################################################

ADD_SUBDIRECTORY( src )

ENABLE_TESTING()
ADD_SUBDIRECTORY( tests )
