################################################################################
# Run the test scripts with CTest
################################################################################

CORAL_GET_PATH_STRING( coralPathStr )

CORAL_ADD_TEST( tests testkit.Run ${CMAKE_CURRENT_SOURCE_DIR} --shuffle -o "output/TestResults$<CONFIGURATION>.xml" )

CORAL_TEST_ENVIRONMENT( tests )
