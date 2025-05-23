# Create interface for core
add_library(
  ${PROJECT_NAME}_core
  src/common.cpp
  src/contact_managers_plugin_factory.cpp
  src/continuous_contact_manager.cpp
  src/discrete_contact_manager.cpp
  src/serialization.cpp
  src/types.cpp
  src/utils.cpp
  src/contact_result_validator.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_geometry
         boost_plugin_loader::boost_plugin_loader
         Boost::boost
         Boost::system
         Boost::serialization
         yaml-cpp
         octomap)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME}_core
                           PRIVATE TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH="${TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH}")
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_core
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT core
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

configure_component(
  COMPONENT core
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core
  DEPENDENCIES
    Eigen3
    console_bridge
    tesseract_geometry
    tesseract_common
    yaml-cpp
    octomap
    boost_plugin_loader
    "Boost COMPONENTS system program_options serialization"
  CFG_EXTRAS cmake/core-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "liboctomap-dev"
      "libboost-system-dev"
      "libboost-program-options-dev"
      "libboost-serialization-dev"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console_bridge"
      "Eigen3"
      "yaml-cpp"
      "liboctomap-dev"
      "libboost-system-dev"
      "libboost-program-options-dev"
      "libboost-serialization-dev"
      "boost-plugin-loader"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")
endif()
