

gtest_begin( lsdslam NO_TEMPLATE )

  set( TESTIMAGES_SRCS ../testdata/testimages.cpp
                       ../testdata/testimages_data.cpp )

  fips_include_directories(../testdata)

    fips_files(
      test_main.cpp
      test_FrameData.cpp
      test_Frame.cpp
      ../testdata/test_testimages.cpp
      ${TESTIMAGES_SRCS}
    )

    fips_deps( lsdslam videoio )
gtest_end()
