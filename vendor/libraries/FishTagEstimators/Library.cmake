# Base folder --------------------------------------------------------------
file(GLOB DUNE_FISHTAGESTIMATOR_FILES
  user/vendor/libraries/FishTagEstimators/*.cpp)

set_source_files_properties(${DUNE_FISHTAGESTIMATOR_FILES}
PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)

list(APPEND DUNE_VENDOR_FILES ${DUNE_FISHTAGESTIMATOR_FILES})
# XKF folder --------------------------------------------------------------
file(GLOB DUNE_FISHTAGESTIMATOR_FILES_XKF
user/vendor/libraries/FishTagEstimators/XKF/*.cpp)

set_source_files_properties(${DUNE_FISHTAGESTIMATOR_FILES_XKF}
PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries/XKF)

list(APPEND DUNE_VENDOR_FILES ${DUNE_FISHTAGESTIMATOR_FILES_XKF})

# XKF folder --------------------------------------------------------------
file(GLOB DUNE_FISHTAGESTIMATOR_FILES_XKFVEL
user/vendor/libraries/FishTagEstimators/XKFVelocity/*.cpp)

set_source_files_properties(${DUNE_FISHTAGESTIMATOR_FILES_XKFVEL}
PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries/XKFVelocity)

list(APPEND DUNE_VENDOR_FILES ${DUNE_FISHTAGESTIMATOR_FILES_XKFVEL})
