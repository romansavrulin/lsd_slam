cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam)
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option(BUILD_GUI "Build Pangolin-based GUI" ON)

## Enable conan tools
option(USE_CONAN  "Use the Conan packaging system" OFF)
if(USE_CONAN)
  include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  conan_basic_setup()

  if( BUILD_GUI )
    ## Check for dependencies for Pangolin
    find_package( GLEW REQUIRED )
    find_package( GLUT REQUIRED )
    find_package( GLM REQUIRED )  # n.b. we provide the FindGLM.cmake file
    find_package( OpenGL REQUIRED )
    find_package( JPEG REQUIRED )
    find_package( PNG REQUIRED )
    find_package( PythonLibs REQUIRED )
    find_package( OpenEXR QUIET )

    set( Pangolin_LIBRARIES
          ${OPENGL_LIBRARIES}
          ${JPEG_LIBRARIES}
          ${PNG_LIBRARIES}
          ${PYTHON_LIBRARIES}
          ${GLEW_LIBRARIES}
          ${OpenEXR_LIBRARIES} )

    if(APPLE)
      ## Pangolin prefers FreeGLUT over the GLUT Framework from XCode
      list( APPEND Pangolin_LIBRARIES glut  )

      include_directories( /usr/local/include )
      link_directories( /usr/local/lib )
    else()
      list( APPEND Pangolin_LIBRARIES  ${GLUT_LIBRARIES}  )
    endif()
  endif()

else()
  ## Non-conan / cmake-based build
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
	set(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

	include( ${PROJECT_SOURCE_DIR}/cmake/BuildG3Log.cmake )
	set( EXTERNAL_PROJECTS g3log )

	include( ${PROJECT_SOURCE_DIR}/cmake/BuildLibActiveObject.cmake )

  option( USE_OPENMP "Enable OpenMP (currently only in local copy of G2O)" OFF )

  ##==== Option to build local copy of G2O ============================
  option( BUILD_LOCAL_G2O "Build a local copy of G2O" ON )
  if( BUILD_LOCAL_G2O )
  	set( G2O_USE_OPENMP ${USE_OPENMP} )
  	list( APPEND EXTERNAL_PROJECTS G2O )

  	include( ${PROJECT_SOURCE_DIR}/cmake/BuildG2O.cmake )
  elseif()
  	find_package(G2O REQUIRED)

  	# These are the G2O libraries I use
  	set( G2O_LIBRARIES
  		${G2O_CORE_LIBRARY}
  		${G2O_STUFF_LIBRARY}
  		${G2O_SOLVER_CSPARSE_EXTENSION}
  		${G2O_SOLVER_CSPARSE}
  		${G2O_TYPES_SBA}
  		${G2O_TYPES_SIM3} )
  endif()

  if(BUILD_GUI)
    ##====  Build local copy of Pangolin? ======================
    option( BUILD_LOCAL_PANGOLIN "Build a local copy of Pangolin" ON )
    if( BUILD_LOCAL_PANGOLIN )
    	list( APPEND EXTERNAL_PROJECTS Pangolin )
    	include( ${PROJECT_SOURCE_DIR}/cmake/BuildPangolin.cmake )
    elseif()
    	find_package(Pangolin 0.1 REQUIRED)
    endif()
  endif()

endif()

## I haven't packaged suitesparse yet...
find_package(SuiteParse REQUIRED)
list( APPEND G2O_LIBRARIES
  ${CSPARSE_LIBRARY} )
list( APPEND G2O_INCLUDE_DIR
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR} )


## Use ccache if available
option( USE_CCACHE "Use CCache (if available)" OFF )
	if( USE_CCACHE )
	find_program(CCACHE_FOUND ccache)
	if(CCACHE_FOUND)
			message( "Using ccache" )
	    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
	endif(CCACHE_FOUND)
endif()


find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)

## Command used to build the external projects
set( EXTERNAL_PROJECT_PARALLELISM 4 CACHE STRING "Level of parallelism when making ExternalProjects" )
set( EXTERNAL_PROJECT_MAKE_COMMAND "make" )
if( EXTERNAL_PROJECT_PARALLELISM GREATER 0 )
	message( "Building ExternalProjects in Parallel, to disable set EXTERNAL_PROJECT_PARALLELISM to 0")
	set( EXTERNAL_PROJECT_MAKE_COMMAND make -j${EXTERNAL_PROJECT_PARALLELISM} )
endif()

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )



# To include/link G2O you need to be able to find the Suitesparse libs



##==== Check and define the architecture
## Architecture-specific optimizations
option( CUDA_VERSION "Use CUDA" "")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	# Jetson version uses OpenCV4Tegra from NVidia
	find_package(OpenCV 2.4 COMPONENTS core highgui imgproc calib3d REQUIRED)

	## For compilation on Jetson TX1
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )

elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
	# if( ZED_VERSION VERSION_EQUAL "1.0.0")
	# 	find_package(OpenCV 3.1 COMPONENTS core highgui imgproc calib3d videoio imgcodecs REQUIRED)
	# else()
		find_package(OpenCV 2.4 COMPONENTS core highgui imgproc calib3d REQUIRED)
#	endif()

	# add_definitions( -DZED_1_0 )

  # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
	# it has nothing to do with compiler behavior
  set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
endif()

if( CUDA_VERSION )
	find_package( CUDA REQUIRED ${CUDA_VERSION} )
  #if(${CUDA_VERSION} VERSION_EQUAL "8.0")
  	SET( CUDA_USE_STATIC_CUDA_RUNTIME OFF )
  #endif()
endif()


## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


set( LSDSLAM_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/lsd_core
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${Pangolin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/thirdparty/Sophus
	${G2O_INCLUDE_DIR} )

if(APPLE)
  include_directories( /usr/local/include )
  link_directories( /usr/local/lib )
endif()

if( USE_CONAN )

	## Set global include paths
	list( APPEND LSDSLAM_INCLUDE_DIRS
	  ${CONAN_INCLUDE_DIRS}
	)

else()

	if(APPLE)
		set( CMAKE_MACOSX_RPATH OFF )
	endif()

	## Set global include paths
	list( APPEND LSDSLAM_INCLUDE_DIRS
	  ${G3LOG_INCLUDE_DIR}
		${LIBACTIVE_OBJECT_INCLUDE_DIRS}
	)
endif()
include_directories( ${LSDSLAM_INCLUDE_DIRS} )

# link_directories( ${ZED_LIBRARY_DIRS}
#  									${CUDA_LIBRARY_DIRS} )

add_subdirectory( lib/ )
add_subdirectory( tools/ )

option( BUILD_UNIT_TESTS "Build unit tests" OFF )
if( BUILD_UNIT_TESTS )
  ## Build a local copy of gtest (as per the GTest FAQ)
	if( NOT USE_CONAN )
  	include( ${CMAKE_SOURCE_DIR}/cmake/BuildGTestLibraries.cmake )
		list( APPEND EXTERNAL_PROJECTS gtest )
	endif()

  add_subdirectory( test/unit/ )
endif()

# Custom target to build dependencies
add_custom_target( deps DEPENDS ${EXTERNAL_PROJECTS} )
