# Copyright (C) 2021, Xilinx Inc - All rights reserved
# Xilinx Multiscaler XMA Plugin 
                                    
# Licensed under the Apache License, Version 2.0 (the "License"). You may
# not use this file except in compliance with the License. A copy of the
# License is located at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations 
# under the License.

cmake_minimum_required(VERSION 2.8.11)
include (CMakeDependentOption)

set(XMAMSCALER_PROJ "xilinx-u30-xma-multiscaler")
set(XMAMSCALER_LIBNAME "xmultiscaler")
set(XMAMSCALER_PROJ_VERSION "1.1.0")

find_program(UNAME_EXEC uname)
find_program(LSB_RELEASE_EXEC lsb_release)

# Grab Arch information like x86-64
execute_process(COMMAND ${UNAME_EXEC} -p
    OUTPUT_VARIABLE UNAME_ARCHTYPE
    OUTPUT_STRIP_TRAILING_WHITESPACE)

# Grab OS release short name
execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
	OUTPUT_VARIABLE LSB_RELEASE_REL_SHORT
	OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CPACK_PACKAGE_FILE_NAME "${XMAMSCALER_PROJ}_${XMAMSCALER_PROJ_VERSION}_${LSB_RELEASE_REL_SHORT}-${UNAME_ARCHTYPE}")
set(CPACK_PACKAGE_VERSION ${XMAMSCALER_PROJ_VERSION})
project(${XMAMSCALER_PROJ})

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/lib/pkgconfig")

# Linking variables
find_package(PkgConfig REQUIRED)
pkg_check_modules(XRT REQUIRED	xrt)
pkg_check_modules(XMA REQUIRED libxmaapi)
pkg_check_modules(XVBM REQUIRED xvbm)
pkg_check_modules(THREADS required)

add_library(XVBM_STATIC_LIB	STATIC IMPORTED GLOBAL)
set_target_properties(XVBM_STATIC_LIB
					PROPERTIES
					IMPORTED_LOCATION "/opt/xilinx/xvbm/lib/libxvbm.a")
set_target_properties(XVBM_STATIC_LIB
					PROPERTIES
					INTERFACE_INCLUDE_DIRECTORIES "/opt/xilinx/xvbm/include/")

if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${XMAMSCALER_LIBNAME} PUBLIC "-pthread")
endif()


# Bring the headers into the project
include_directories(include)

# Generate the shared library from the sources
add_library(${XMAMSCALER_LIBNAME} SHARED
	src/xlnx_multi_scaler.cpp
)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS	"${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -DDSA64 -fpermissive")

if (ENABLE_PIPELINE)
  set(CMAKE_CXX_FLAGS	"${CMAKE_CXX_FLAGS} -DNO_PIPELINE")
endif()

target_compile_options(${XMAMSCALER_LIBNAME}
	PUBLIC	${XRT_CFLAGS}
	PUBLIC	${XMA_CFLAGS}
	PUBLIC	${XVBM_CFLAGS}
)

target_include_directories(${XMAMSCALER_LIBNAME}
	PUBLIC ${XRT_INCLUDE_DIRS}/xma2
	PUBLIC ${XRT_INCLUDE_DIRS}
	PUBLIC ${XVBM_INCLUDE_DIRS}
)

target_link_libraries(${XMAMSCALER_LIBNAME} XVBM_STATIC_LIB)

if(CMAKE_THREAD_LIBS_INIT)
	target_link_libraries(${XMAMSCALER_LIBNAME} ${CMAKE_THREAD_LIBS_INIT})
endif()

set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XVBM_LDFLAGS} ${XRT_LDFLAGS} --stats -Bsymbolic")

# Set the location for library installation
install(TARGETS ${XMAMSCALER_LIBNAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/xma_plugins)

# Packaging section
set(CPACK_PACKAGE_CONTACT "Rohit Athavale <rohit.athavale@xilinx.com>")
set(CPACK_PACKAGE_VENDOR "Xilinx")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Xilinx Multi-Scaler Plugin")
set(CPACK_PACKAGE_DESCRIPTION "The Xilinx MPSoC based Scaler Plugin controls
                               the accelerated FPGA scaler kernel used to perform
                               scaling of video. Up to 5 output resolutions can be
                               generated from a single input")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "xrt (>= 2.11), xilinx-xvbm (>= 1.0.2) ")
set(CPACK_RPM_PACKAGE_DEPENDS "xrt (>= 2.11), xilinx-xvbm (>= 1.0.2) ")
set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION "/opt" "/opt/xilinx" "/usr/lib" "/usr/lib/pkgconfig")
include(CPack)
