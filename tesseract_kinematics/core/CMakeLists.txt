add_library(
  ${PROJECT_NAME}_core
  src/inverse_kinematics.cpp
  src/rop_inv_kin.cpp
  src/rep_inv_kin.cpp
  src/joint_group.cpp
  src/kinematic_group.cpp
  src/kinematics_plugin_factory.cpp
  src/utils.cpp
  src/validate.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_scene_graph
         tesseract::tesseract_state_solver_kdl
         console_bridge::console_bridge
         boost_plugin_loader::boost_plugin_loader
         yaml-cpp)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME}_core
                           PRIVATE TESSERACT_KINEMATICS_PLUGIN_PATH="${TESSERACT_KINEMATICS_PLUGIN_PATH}")
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

# Add KDL kinematics factories
add_library(${PROJECT_NAME}_core_factories src/rop_factory.cpp src/rep_factory.cpp)
target_link_libraries(${PROJECT_NAME}_core_factories PUBLIC ${PROJECT_NAME}_core console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_core_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_core_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                        "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "${PROJECT_NAME}_core_factories" PARENT_SCOPE)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT core
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

configure_component(
  COMPONENT core
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_core_factories
  DEPENDENCIES
    Eigen3
    console_bridge
    tesseract_scene_graph
    tesseract_state_solver
    tesseract_common
    boost_plugin_loader
    yaml-cpp
  CFG_EXTRAS cmake/core-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console_bridge"
      "Eigen3"
      "yaml-cpp"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")
endif()
