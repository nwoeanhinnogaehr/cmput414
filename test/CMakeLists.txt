cmake_minimum_required(VERSION 2.8.12)
project(cmput414)

### libIGL options: choose between header only and compiled static library
option(LIBIGL_USE_STATIC_LIBRARY "Use LibIGL as static library" ON)
option(LIBIGL_WITH_VIEWER      "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_NANOGUI     "Use Nanogui menu"   OFF)

### libIGL options: choose your dependencies (by default everything is OFF, in this example we need the viewer) ###
option(LIBIGL_WITH_CGAL             "Use CGAL"           OFF) # This option is not supported yet
option(LIBIGL_WITH_BOOLEAN          "Use Cork boolean"   OFF) # This option is not supported yet
option(LIBIGL_WITH_COMISO           "Use CoMiso"         ON)
option(LIBIGL_WITH_EMBREE           "Use Embree"         ON)
option(LIBIGL_WITH_LIM              "Use LIM"            ON)
option(LIBIGL_WITH_MATLAB           "Use Matlab"         OFF) # This option is not supported yet
option(LIBIGL_WITH_MOSEK            "Use MOSEK"          OFF) # This option is not supported yet
option(LIBIGL_WITH_BBW              "Use BBW"            ON)
option(LIBIGL_WITH_OPENGL_AND_PNG   "Use OpenGL"         ON)
option(LIBIGL_WITH_TETGEN           "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE         "Use Triangle"       ON)
option(LIBIGL_WITH_XML              "Use XML"            ON)
### End   to be tested ----

### libIGL options: decide if you want to use the functionalities that depends on cgal
if(LIBIGL_WITH_CGAL) # Do not remove or move this block, cgal strange build system fails without it
    find_package(CGAL REQUIRED)
    set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "CGAL's CMAKE Setup is super annoying ")
    include(${CGAL_USE_FILE})
endif()

add_subdirectory("${PROJECT_SOURCE_DIR}/../../libigl/shared/cmake" "libigl")


### Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

### Compilation flags: adapt to your needs ###
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj") ### Enable parallel compilation
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR} )
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR} )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w") # disable all warnings (not ideal but...)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") #### Libigl requires a modern C++ compiler that supports c++11
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") # disable all warnings (not ideal but...)
endif()

# Enable openMP if possible
#find_package(OpenMP)
#if (OPENMP_FOUND AND NOT WIN32)
#  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()


### Prepare the build environment

include_directories(${LIBIGL_INCLUDE_DIRS})
add_definitions(${LIBIGL_DEFINITIONS})

### Prepare the build environment

add_executable(${PROJECT_NAME}_bin
    main.cpp collapse_edge.cpp)
include_directories(${LIBIGL_INCLUDE_DIRS})
add_definitions(${LIBIGL_DEFINITIONS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(${PROJECT_NAME}_bin PRIVATE ${LIBIGL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}_bin PRIVATE ${LIBIGL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_bin ${LIBIGL_LIBRARIES} ${LIBIGL_EXTRA_LIBRARIES})

