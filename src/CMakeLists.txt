################################################################################
# Build the Module
################################################################################

coral_generate_module( _MODULE_SOURCES eigen )

include_directories(
	${CORAL_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}/generated
)

# The below definitions yield compile problems under vc2010
#set_property( DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "EIGEN_DONT_VECTORIZE" )
#set_property( DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT" )
#set_property( DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "EIGEN_DONT_ALIGN_STATICALLY" )

file( GLOB _SOURCE_FILES *.cpp )
file( GLOB _HEADER_FILES *.hpp )

add_library( eigen MODULE ${_HEADER_FILES} ${_SOURCE_FILES} ${_MODULE_SOURCES} )

target_link_libraries( eigen ${CORAL_LIBRARIES} )

coral_module_target( "eigen" eigen )

################################################################################
# Source Groups
################################################################################

source_group( "@Generated" FILES ${_MODULE_SOURCES} )
