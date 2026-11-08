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

# Vehicle models linked into the runtime: shim directories written by
# `python -m cp_vehicle_interface prepare MODEL.fmu OUT_DIR --shim NAME`.
# Each model is compiled from its FMU's C sources (as the FMU's own binary is:
# C99, -O2) and partially linked into one object in which only cpv_model_NAME
# stays global, so any number of models link into one program. Hosts find them
# with cpv_find_model(NAME) and open them with cpv_open_model.
set(CPV_MODELS "" CACHE STRING "Shim directories (prepare --shim) to link into the runtime")
set(CPV_MODEL_OBJECTS)
set(CPV_MODEL_NAMES)
set(CPV_REGISTRY_DECLARATIONS "")
set(CPV_REGISTRY_ENTRIES "")
if(CPV_MODELS)
  if(NOT CMAKE_OBJCOPY)
    message(FATAL_ERROR "CPV_MODELS needs objcopy (GNU binutils) to localize model symbols")
  endif()
endif()
foreach(shim IN LISTS CPV_MODELS)
  get_filename_component(shim "${shim}" ABSOLUTE)
  if(NOT EXISTS "${shim}/cpv_model.cmake")
    message(FATAL_ERROR "${shim} is not a shim directory (prepare ... --shim NAME writes OUT_DIR/shim)")
  endif()
  unset(CPV_MODEL_NAME)
  include("${shim}/cpv_model.cmake")
  set(name ${CPV_MODEL_NAME})
  if(name IN_LIST CPV_MODEL_NAMES)
    message(FATAL_ERROR "CPV_MODELS: two models are named ${name}")
  endif()
  list(APPEND CPV_MODEL_NAMES ${name})
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${shim}/cpv_model.cmake")
  list(TRANSFORM CPV_MODEL_SOURCES PREPEND "${shim}/")
  add_library(cpv_model_${name}_objects OBJECT "${shim}/cpv_model_${name}.c" ${CPV_MODEL_SOURCES})
  target_include_directories(cpv_model_${name}_objects PRIVATE include third_party/fmi3 "${shim}/sources")
  target_compile_definitions(cpv_model_${name}_objects PRIVATE ${CPV_MODEL_DEFINITIONS})
  set_target_properties(cpv_model_${name}_objects PROPERTIES C_STANDARD 99 C_EXTENSIONS OFF)
  # Generated code: the FMU's own flags, no project warnings.
  target_compile_options(cpv_model_${name}_objects PRIVATE -O2 -w)
  set(object "${CMAKE_CURRENT_BINARY_DIR}/cpv_model_${name}.o")
  add_custom_command(OUTPUT "${object}"
    COMMAND ${CMAKE_LINKER} -r -o "${object}.partial" "$<TARGET_OBJECTS:cpv_model_${name}_objects>"
    COMMAND ${CMAKE_OBJCOPY} --keep-global-symbol=cpv_model_${name} "${object}.partial" "${object}"
    COMMAND ${CMAKE_COMMAND} -E remove "${object}.partial"
    DEPENDS cpv_model_${name}_objects "$<TARGET_OBJECTS:cpv_model_${name}_objects>"
    COMMENT "Linking vehicle model ${name}"
    COMMAND_EXPAND_LISTS VERBATIM)
  # One target drives the link step; the runtime libraries depend on it so the
  # rule never runs twice in parallel.
  add_custom_target(cpv_model_${name} DEPENDS "${object}")
  list(APPEND CPV_MODEL_OBJECTS "${object}")
  string(APPEND CPV_REGISTRY_DECLARATIONS "extern const cpv_model_t cpv_model_${name};\n")
  string(APPEND CPV_REGISTRY_ENTRIES "  &cpv_model_${name},\n")
endforeach()
configure_file(src/models.c.in "${CMAKE_CURRENT_BINARY_DIR}/cpv_models.c" @ONLY)
set_source_files_properties(${CPV_MODEL_OBJECTS} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)
if(CPV_MODEL_NAMES)
  message(STATUS "cpvehicle: linked models ${CPV_MODEL_NAMES}")
endif()

set(CPV_SOURCES src/vehicle.c src/json.c src/sha256.c "${CMAKE_CURRENT_BINARY_DIR}/cpv_models.c"
    ${CPV_MODEL_OBJECTS})
add_library(cpvehicle_static STATIC ${CPV_SOURCES})
add_library(cpvehicle SHARED ${CPV_SOURCES})
foreach(target cpvehicle cpvehicle_static)
  foreach(name IN LISTS CPV_MODEL_NAMES)
    add_dependencies(${target} cpv_model_${name})
  endforeach()
  target_include_directories(${target} PUBLIC include PRIVATE src third_party/fmi3)
  target_link_libraries(${target} PUBLIC ${CMAKE_DL_LIBS} m)
endforeach()
set_target_properties(cpvehicle_static PROPERTIES OUTPUT_NAME cpvehicle)

add_executable(cpv_hover_host examples/hover_host.c)
target_link_libraries(cpv_hover_host PRIVATE cpvehicle_static)
add_executable(cpv_realtime examples/realtime_probe.c)
target_link_libraries(cpv_realtime PRIVATE cpvehicle_static)
add_executable(cpv_sensors examples/sensor_probe.c)
target_link_libraries(cpv_sensors PRIVATE cpvehicle_static)

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
  set(CPV_TEST_VEHICLES "" CACHE PATH "Directory holding prepared quad/, hex/, rover/, plane/ and legacy/ descriptors")
  add_executable(test_ground_air tests/test_ground_air.c)
  target_link_libraries(test_ground_air PRIVATE cpvehicle_static m)
  add_executable(test_linked_models tests/test_linked_models.c)
  target_link_libraries(test_linked_models PRIVATE cpvehicle_static m)
  if(CPV_TEST_VEHICLES)
    add_test(NAME conformance COMMAND test_conformance ${CPV_TEST_VEHICLES})
    add_test(NAME ground_air COMMAND test_ground_air ${CPV_TEST_VEHICLES})
    if(CPV_MODEL_NAMES)
      add_test(NAME linked_models COMMAND test_linked_models ${CPV_TEST_VEHICLES})
    endif()
  endif()
endif()
