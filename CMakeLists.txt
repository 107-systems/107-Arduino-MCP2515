##########################################################################
cmake_minimum_required(VERSION 3.28)
##########################################################################
project("libmcp2515")
##########################################################################
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_LESS 11)
  message(WARNING "GNU C compiler version ${CMAKE_C_COMPILER_VERSION} is too old and is unsupported. Version 11+ is required.")
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
  message(WARNING "GNU C++ compiler version ${CMAKE_CXX_COMPILER_VERSION} is too old and is unsupported. Version 11+ is required.")
endif()
##########################################################################
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
##########################################################################
add_library(${PROJECT_NAME} STATIC
    "src/ArduinoMCP2515.cpp"
    "src/MCP2515/MCP2515_Config.cpp"
    "src/MCP2515/MCP2515_Const.cpp"
    "src/MCP2515/MCP2515_Control.cpp"
    "src/MCP2515/MCP2515_Io.cpp"
)
##########################################################################
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
##########################################################################
target_include_directories(${PROJECT_NAME} PUBLIC
    "src"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
##########################################################################
