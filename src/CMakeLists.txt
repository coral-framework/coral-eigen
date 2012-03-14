################################################################################
# Build the Module
################################################################################

CORAL_GENERATE_MODULE( _MODULE_SOURCES eigen )

INCLUDE_DIRECTORIES(
	${CORAL_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/include/eigen/
	${CMAKE_CURRENT_BINARY_DIR}/generated
)

set_property( DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "EIGEN_DONT_VECTORIZE" )
set_property( DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT" )
set_property( DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "EIGEN_DONT_ALIGN_STATICALLY" )

FILE( GLOB _SOURCE_FILES *.cpp )
FILE( GLOB _HEADER_FILES *.hpp )

ADD_LIBRARY( eigen MODULE ${_HEADER_FILES} ${_SOURCE_FILES} ${_MODULE_SOURCES} )

TARGET_LINK_LIBRARIES( eigen ${CORAL_LIBRARIES} )

CORAL_MODULE_TARGET( "eigen" eigen )

################################################################################
# Source Groups
################################################################################

SOURCE_GROUP( "@Generated" FILES ${_MODULE_SOURCES} )
