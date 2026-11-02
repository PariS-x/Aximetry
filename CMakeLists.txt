cmake_minimum_required(VERSION 3.16)
project(aximetry VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_options(/W4 /permissive- /D_CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Core simulator library (no third-party dependencies)
add_library(aximetry_core STATIC
  src/json.cpp
  src/terramechanics.cpp
  src/materials.cpp
  src/environment.cpp
  src/rover.cpp
  src/design_check.cpp
  src/terrain.cpp
  src/mission.cpp
  src/controller.cpp
  src/sim.cpp
  src/replay.cpp
  src/gui_server.cpp
)
target_include_directories(aximetry_core PUBLIC include)

# ---- Controllers: every .cpp under controllers/ is compiled in automatically.
# They self-register, so they are built as an OBJECT library and linked directly
# (a static library would let the linker drop the registration objects).
file(GLOB_RECURSE AXIMETRY_CONTROLLER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/controllers/*.cpp)
add_library(aximetry_controllers OBJECT ${AXIMETRY_CONTROLLER_SOURCES})
target_link_libraries(aximetry_controllers PUBLIC aximetry_core)

# ---- Command-line tool
add_executable(aximetry src/main.cpp $<TARGET_OBJECTS:aximetry_controllers>)
target_link_libraries(aximetry PRIVATE aximetry_core)
target_compile_definitions(aximetry PRIVATE AXIMETRY_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# ---- "Run your own code" from the GUI: the same command-line front end as a
# library, so the GUI can compile one controller file and link it against
# these two prebuilt libraries in a few seconds, without rebuilding everything.
add_library(aximetry_cli STATIC src/main.cpp)
target_link_libraries(aximetry_cli PUBLIC aximetry_core)
target_compile_definitions(aximetry_cli PRIVATE AXIMETRY_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_dependencies(aximetry aximetry_cli)
target_compile_definitions(aximetry PRIVATE
  AXIMETRY_GUI_COMPILER="${CMAKE_CXX_COMPILER}"
  AXIMETRY_GUI_COMPILER_ID="${CMAKE_CXX_COMPILER_ID}"
  AXIMETRY_GUI_CLI_LIB="$<TARGET_FILE:aximetry_cli>"
  AXIMETRY_GUI_CORE_LIB="$<TARGET_FILE:aximetry_core>"
  AXIMETRY_GUI_BUILD_DIR="${CMAKE_BINARY_DIR}")

# The GUI's local web server needs the Windows socket library.
if(WIN32)
  target_link_libraries(aximetry_core PUBLIC ws2_32)
endif()


# ---- Tests
enable_testing()
add_executable(aximetry_tests tests/test_main.cpp $<TARGET_OBJECTS:aximetry_controllers>)
target_link_libraries(aximetry_tests PRIVATE aximetry_core)
target_compile_definitions(aximetry_tests PRIVATE AXIMETRY_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME aximetry_tests COMMAND aximetry_tests)

# MinGW: link the C++ runtime statically so the .exe files run on double-click
# without needing MinGW's DLLs on the PATH.
if(MINGW)
  target_link_options(aximetry PRIVATE -static)
  target_link_options(aximetry_tests PRIVATE -static)
endif()
