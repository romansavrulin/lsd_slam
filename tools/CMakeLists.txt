
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# set(CMAKE_CXX_FLAGS
#    "${CMAKE_CXX_FLAGS} -ggdb"
# )

link_directories( ${CONAN_LIB_DIRS} )

set( LSD_SRCS
  LSD/InputThread.cpp
  LSD/ParseArgs.cpp )

# build image node
add_executable(LSD LSD.cpp
  ${LSD_SRCS}  )
target_link_libraries( LSD lsdslam ${CONAN_LIBS} )

if(BUILD_GUI)
  add_executable(LSD_GUI LSD_GUI.cpp
    LSD_GUI/GuiThread.cpp
    LSD_GUI/GUI.cpp
    LSD_GUI/Pangolin_IOWrapper/PangolinOutput3DWrapper.cpp
    ${LSD_SRCS} )
  target_link_libraries( LSD_GUI lsdslam ${CONAN_LIBS} ${Pangolin_LIBRARIES} )
endif()
