##########################################################
# SQLite 3 RTREE option
# This is needed when spatialite uses the Rtree
##########################################################

set(SQLITE3_C_FLAGS "${SQLITE3_C_FLAGS} -DSQLITE_ENABLE_RTREE=1")

set_source_files_properties(${DUNE_SQLITE3_FILES}
  PROPERTIES COMPILE_FLAGS "${DUNE_C_FLAGS} ${SQLITE3_C_FLAGS}")

