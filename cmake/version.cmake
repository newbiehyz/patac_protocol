# ##############################################################################
# build version
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)

set(SONAME_MAJOR 1) # should changes when APIs changes(which making things incompatible).
set(SONAME_MINOR 31) # should changes by adding APIs.
set(SONAME_PATCH 6) # should changes by bug fixing or optimizing

set(PROJECT_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

execute_process(COMMAND git log --pretty=format:"%h" -n 1
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  TIMEOUT 3
  RESULT_VARIABLE RESULT_OUTPUT
  OUTPUT_VARIABLE COMMIT_OUTPUT
)

set(PROJECT_COMPILE_COMMIT ${COMMIT_OUTPUT})

string(TIMESTAMP COMPILE_TIME %Y%m%d_%H%M)
string(RANDOM LENGTH 6 RANDOM_VERSION_MD)
string(TOLOWER ${RANDOM_VERSION_MD} RANDOM_VERSION_MD)
set(VERSION "{\"version\": \"v${PROJECT_VERSION}_${COMPILE_TIME}_${RANDOM_VERSION_MD}\", \"commit\": ${PROJECT_COMPILE_COMMIT}, \"platform\": \"all\"}\n")
file(WRITE "${PROJECT_SOURCE_DIR}/build/version.json" ${VERSION})
