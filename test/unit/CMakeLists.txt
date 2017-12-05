

gtest_begin( lsdslam )

    fips_files(
      test_test.cpp
    )

    fips_libs(lsdslam)

    fips_deps(videoio)
gtest_end()
