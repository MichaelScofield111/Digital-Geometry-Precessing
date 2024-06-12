cmake_minimum_required(VERSION 3.5)

project(ACM_mesh)

#设置不生成MinSizeRel;RelWithDebInfo
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

#Generate the static library from the library sources
add_library(ACAM_mesh STATIC 
    src/IOManger.cpp
	src/PolyMesh.cpp
	src/PolyMesh_Base.cpp
	src/PolyMeshIterators.cpp
)

#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) #设置库文件的输出目录
# when you Compile ACAM_mesh, you should find head.h in ${PROJECT_SOURCE_DIR}/include
target_include_directories(ACAM_mesh
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_BINARY_DIR}/src/hw_1
)

# This line of code defines an executable target named ACAM_example and 
# specifies the source files used to build the executable.
add_executable(ACAM_example src/hw_1/main.cpp src/hw_1/Algorithm.cpp)

# link the new ACAM_example target with the ACAM_mesh target
target_link_libraries( ACAM_example
    PRIVATE 
        ACAM_mesh
)
