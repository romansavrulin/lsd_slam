
fips_begin_app(LSD cmdline)

  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

  fips_files( LSD.cpp
                ParseArgs.cpp  )
  fips_libs( lsdslam )
fips_end_app()

# if(BUILD_GUI)
#   add_executable(LSD_GUI LSD_GUI.cpp
#     LSD_GUI/GuiThread.cpp
#     LSD_GUI/GUI.cpp
#     LSD_GUI/Pangolin_IOWrapper/PangolinOutput3DWrapper.cpp
#     LSD_GUI/Pangolin_IOWrapper/PangolinOutputIOWrapper.cpp
#     ${LSD_SRCS} )
#   target_link_libraries( LSD_GUI lsdslam ${CONAN_LIBS} ${Pangolin_LIBRARIES} )
# endif()
