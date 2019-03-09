

gtest_begin( lsdslam )

    fips_files(
      test_test.cpp
      test_FrameData.cpp
      test_Frame.cpp
    )

    fips_deps( lsdslam videoio )
gtest_end()
