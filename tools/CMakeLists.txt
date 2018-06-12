
## TODO.  Strip GUI elements from this repo.

fips_begin_app(LSD cmdline)
  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

  fips_files( LSD.cpp
                ParseArgs.cpp  )

  fips_deps( lsdslam videoio g3logger g3log )
fips_end_app()
