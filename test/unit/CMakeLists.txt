include_directories( ${GTEST_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/lib )

add_executable( lsdslam_test
              test_test.cpp )

target_link_libraries( lsdslam_test
          ${GTEST_BOTH_LIBRARIES}
          lsdslam )

# # Don't use ctest, instead just execture the Google test binaries
add_custom_target( unit_test
                		WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
                		COMMAND ./lsdslam_test
                    DEPENDS lsdslam_test )
