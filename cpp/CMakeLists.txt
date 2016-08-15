cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(multiblock_admm)
###########################################################################

# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
include(GtsamTesting)
###########################################################################

# Find Boost
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
###########################################################################

# ADMM
file(GLOB srcs "src/*.cpp" "src/*.h")
include_directories(src)
add_library (admm SHARED ${srcs} ${GTSAM_LIBRARIES})
target_link_libraries(admm ${Boost_LIBRARIES} gtsam)

###########################################################################

# Executables
add_executable (PartitionAndRunADMM  apps/PartitionAndRunADMM)
target_link_libraries (PartitionAndRunADMM admm boost_system gtsam)

# Deprecated
add_executable (RunADMM  apps/RunADMM.cpp)
target_link_libraries (RunADMM admm boost_system gtsam)
###########################################################################

#Tests
add_subdirectory(tests)
