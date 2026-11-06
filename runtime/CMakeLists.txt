cmake_minimum_required(VERSION 3.16)
project(cpvehicle VERSION 1.0.0 LANGUAGES C)

# Self-contained: no dependency beyond libc, libm and libdl.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(CPV_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(CPV_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

set(CPV_SOURCES src/vehicle.c src/json.c src/sha256.c)
add_library(cpvehicle_static STATIC ${CPV_SOURCES})
add_library(cpvehicle SHARED ${CPV_SOURCES})
foreach(target cpvehicle cpvehicle_static)
  target_include_directories(${target} PUBLIC include PRIVATE src third_party/fmi3)
  target_link_libraries(${target} PUBLIC ${CMAKE_DL_LIBS} m)
endforeach()
set_target_properties(cpvehicle_static PROPERTIES OUTPUT_NAME cpvehicle)

add_executable(cpv_hover_host examples/hover_host.c)
target_link_libraries(cpv_hover_host PRIVATE cpvehicle_static)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_internals tests/test_internals.c)
  target_include_directories(test_internals PRIVATE src)
  target_link_libraries(test_internals PRIVATE cpvehicle_static)
  add_test(NAME internals COMMAND test_internals)

  add_executable(test_conformance tests/test_conformance.c)
  target_link_libraries(test_conformance PRIVATE cpvehicle_static)
  # Conformance tests run against prepared descriptors; tests/run_conformance.sh
  # builds the FMUs and descriptors and passes this directory.
  set(CPV_TEST_VEHICLES "" CACHE PATH "Directory holding prepared quad/, hex/ and legacy/ descriptors")
  if(CPV_TEST_VEHICLES)
    add_test(NAME conformance COMMAND test_conformance ${CPV_TEST_VEHICLES})
  endif()
endif()
