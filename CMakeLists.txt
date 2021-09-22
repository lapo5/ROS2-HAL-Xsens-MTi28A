cmake_minimum_required(VERSION 3.3)
project(hal_xsens_mti_28a)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_definitions(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_library(cmt STATIC 
    src/libcmt/cmt1.cpp  
    src/libcmt/cmt2.cpp  
    src/libcmt/cmt3.cpp  
    src/libcmt/cmtmessage.cpp  
    src/libcmt/cmtpacket.cpp  
    src/libcmt/cmtscan.cpp  
    src/libcmt/xsens_std.cpp  
    src/libcmt/xsens_time.cpp)
target_compile_options(cmt PUBLIC -Wall -Wextra -Wno-attributes -Wno-attribute-warning)
target_include_directories(cmt PUBLIC include/libcmt)
target_link_libraries(cmt -lrt)

# CFLAGS=-I. -Wall -Wno-attributes
# LFLAGS=-lrt
# DEPS = cmt1.h
# OBJSTATIC = cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o
#
# all: static
#
# %.o: %.cpp $(DEPS)
#	 $(CC) $(CFLAGS) -c -o $@ $<
#
# static: $(OBJSTATIC)
#	 ar rcs libcmt.a $^

execute_process(COMMAND python3-config --extension-suffix OUTPUT_VARIABLE PYBIND11_SUFFIX)
string(STRIP ${PYBIND11_SUFFIX} PYBIND11_SUFFIX)

execute_process(COMMAND python3 -m pybind11 --includes OUTPUT_VARIABLE PYBIND11_INCLUDES)
string(STRIP ${PYBIND11_INCLUDES} PYBIND11_INCLUDES)
string(REPLACE "-I" " " PYBIND11_INCLUDES_LIST ${PYBIND11_INCLUDES})
string(STRIP ${PYBIND11_INCLUDES_LIST} PYBIND11_INCLUDES_LIST)
string(REPLACE " " ";" PYBIND11_INCLUDES_LIST ${PYBIND11_INCLUDES_LIST})

add_library(xsens_MTi28A SHARED src/xsens_MTi28A.cpp)
set_target_properties(xsens_MTi28A PROPERTIES PREFIX "")  
set_target_properties(xsens_MTi28A PROPERTIES SUFFIX ${PYBIND11_SUFFIX})
target_compile_options(cmt PUBLIC -O3 -mtune=native -march=native -DNDEBUG -Wall -Wextra -Wno-attributes)
target_include_directories(cmt PUBLIC include "$<BUILD_INTERFACE:${PYBIND11_INCLUDES_LIST}>")
target_link_libraries(xsens_MTi28A cmt)

#	g++ -O3 -mtune=native -march=native -DNDEBUG -Wall -Wextra -Wno-attributes -shared -std=c++17 -fPIC \
#	`python3 -m pybind11 --includes` \
#	-isystem../dependencies/install/include \
#	xsens_MTi28A.cpp -o xsens_MTi28A`python3-config --extension-suffix` \
#	-L../dependencies/install/lib -lcmt


# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# Install python hal_xsens_mti_28a
install(PROGRAMS hal_xsens_mti_28a/imu_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/libcmt
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(
  TARGETS cmt
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS xsens_MTi28A
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclpy)
ament_export_libraries(${PROJECT_NAME})

ament_package()