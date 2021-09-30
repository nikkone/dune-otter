############################################################################
# Copyright 2013-2021 Norwegian University of Science and Technology (NTNU)#
# Department of Engineering Cybernetics (ITK)                              #
############################################################################
############################################################################
# Author: Nikolai Lauvås                                                   #
############################################################################

find_package(ompl)

if(NOT ompl_FOUND)
  set(DUNE_USING_OMPL 0)
  message("ompl not found")
else()
  #Include code for using OMPL in DUNE
  file(GLOB DUNE_OMPL_FILES
  user/vendor/libraries/OMPL/*.cpp)
  set_source_files_properties(${DUNE_OMPL_FILES}
  PROPERTIES COMPILE_FLAGS "${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")

  list(APPEND DUNE_VENDOR_FILES ${DUNE_OMPL_FILES})

  set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)
  #Add OMPL libraries to DUNE build
  include_directories(${OMPL_INCLUDE_DIRS})
  link_directories(${OMPL_LIBRARY_DIRS})
  dune_add_lib(${OMPL_LIBRARIES} -lboost_system)
  set(DUNE_USING_OMPL 1)
endif()