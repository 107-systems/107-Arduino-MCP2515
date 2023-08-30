cmake_minimum_required(VERSION 3.27.0)

# Create Project
project(107_arduino_mcp2515)

add_library(${PROJECT_NAME} STATIC
    "src/ArduinoMCP2515.cpp"
    "src/MCP2515/MCP2515_Config.cpp"
    "src/MCP2515/MCP2515_Const.cpp"
    "src/MCP2515/MCP2515_Control.cpp"
    "src/MCP2515/MCP2515_Io.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "src"
)
