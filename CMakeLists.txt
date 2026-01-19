cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RMtextH7)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

LINK_LIBRARIES(m)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 查找source目录下的所有*.cpp,*.c源文件,并将文件列表保存到 DIR_LIB_SRCS 变量
file(GLOB_RECURSE SRC_DIR_LIST CONFIGURE_DEPENDS "source/*.cpp" "source/*.c")	#遍历获取source/路径下所有的*.cpp和*.c文件列表
#打印SRC_DIR_LIST中列表信息
message(STATUS "SRC_DIR_LIST = ${SRC_DIR_LIST}")	

# file(GLOB_RECURSE DSP_DIR_LIST "Drivers/CMSIS/DSP/Source/*.cpp" "Drivers/CMSIS/DSP/Source/*.c")	#遍历获取source/路径下所有的*.cpp和*.c文件列表
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
	${SRC_DIR_LIST}
	Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c
	Drivers/CMSIS/DSP/Source/BayesFunctions/BayesFunctions.c
	Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c
	Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c
	Drivers/CMSIS/DSP/Source/ControllerFunctions/ControllerFunctions.c
	Drivers/CMSIS/DSP/Source/DistanceFunctions/DistanceFunctions.c
	Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c
	Drivers/CMSIS/DSP/Source/FilteringFunctions/FilteringFunctions.c
	Drivers/CMSIS/DSP/Source/InterpolationFunctions/InterpolationFunctions.c
	Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c
	Drivers/CMSIS/DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c
	Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c
	Drivers/CMSIS/DSP/Source/SupportFunctions/SupportFunctions.c
	Drivers/CMSIS/DSP/Source/SVMFunctions/SVMFunctions.c
	Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c
	# Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_f32.c
)
 
macro(FIND_INCLUDE_DIR result curdir)									#定义函数,2个参数:存放结果result；指定路径curdir；
	file(GLOB_RECURSE children "${curdir}/*.hpp" "${curdir}/*.h" )		#遍历获取{curdir}中*.hpp和*.h文件列表
	set(dirlist "")														#定义dirlist中间变量，并初始化
	foreach(child ${children})											#for循环
		string(REGEX REPLACE "(.*)/.*" "\\1" LIB_NAME ${child})			#字符串替换,用/前的字符替换/*h
		if(IS_DIRECTORY ${LIB_NAME})									#判断是否为路径
			list (FIND dirlist ${LIB_NAME} list_index)					#去重，查找dirlist中是否有${LIB_NAME}指定的值，可以区分字符串相同数子后缀不同的路径：例如/app/test_1和/app/test_2
			if(${list_index} LESS 0)									#若没找到则代表列表中没有该路径
				LIST(APPEND dirlist ${LIB_NAME})						#将合法的路径加入dirlist变量中  
			endif()														#结束判断
		endif()															
	endforeach()														#结束for循环
	set(${result} ${dirlist})											#dirlist结果放入result变量中
endmacro()

# 查找source目录下的所有*.hpp,*.h源文件,并将文件列表保存到 DIR_LIB_SRCS 变量
FIND_INCLUDE_DIR(DIR_LIB_SRCS ${PROJECT_SOURCE_DIR}/source)
message(STATUS "DIR_LIB_SRCS = ${DIR_LIB_SRCS}")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${DIR_LIB_SRCS}
	Drivers/CMSIS/DSP/Include
	Drivers/CMSIS/DSP/PrivateInclude
)

# #DSP
# add_definitions(-DARM_MATH_CM7 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING)
# include_directories("Drivers/CMSIS/DSP/Include")
# link_directories(${PROJECT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC)
# link_libraries(libarm_cortexM7l_math.a)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
	ARM_MATH_CM7
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
	${PROJECT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a
)
