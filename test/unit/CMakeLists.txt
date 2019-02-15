

gtest_begin( lsdslam )

    fips_files(
      test_test.cpp
    )

    fips_deps( lsdslam videoio )
gtest_end()
