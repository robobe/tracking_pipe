cmake_minimum_required(VERSION 3.28)
project(tracking_pipe_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp REQUIRED)

add_library(bbox_estimator_core src/bbox_estimator.cpp)
target_include_directories(bbox_estimator_core PUBLIC include)
target_compile_options(bbox_estimator_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(bbox_sim src/bbox_sim.cpp)
target_link_libraries(bbox_sim PRIVATE bbox_estimator_core)
target_include_directories(bbox_sim SYSTEM PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
target_compile_options(bbox_sim PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(CTest)
if(BUILD_TESTING)
  add_test(NAME sim_startup COMMAND bbox_sim ${CMAKE_SOURCE_DIR}/tests/fixtures/startup.jsonl --expected ${CMAKE_SOURCE_DIR}/tests/fixtures/startup.expected.jsonl)
  add_test(NAME sim_delayed_yolo COMMAND bbox_sim ${CMAKE_SOURCE_DIR}/tests/fixtures/delayed_yolo.jsonl --expected ${CMAKE_SOURCE_DIR}/tests/fixtures/delayed_yolo.expected.jsonl)
  add_test(NAME sim_bad_yolo COMMAND bbox_sim ${CMAKE_SOURCE_DIR}/tests/fixtures/bad_yolo.jsonl --expected ${CMAKE_SOURCE_DIR}/tests/fixtures/bad_yolo.expected.jsonl)
  add_test(NAME sim_retrack COMMAND bbox_sim ${CMAKE_SOURCE_DIR}/tests/fixtures/retrack.jsonl --expected ${CMAKE_SOURCE_DIR}/tests/fixtures/retrack.expected.jsonl)
endif()
