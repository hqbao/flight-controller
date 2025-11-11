set(VIDEO_DIR video)
set(DISPLAY_DIR display)

set(FOUNDATION_DIR ../../../foundation)
set(MODULE_DIR ../../../../modules)
set(ROBOTKIT_DIR ../../../../libs/robotkit)
set(OPTFLOW_DIR ../../../../libs/optflow)

# Collect app core source files
file(GLOB APP_VIDEO_C_SOURCES ${VIDEO_DIR}/*.c)

# Collect Video module source files  
file(GLOB APP_DISPLAY_C_SOURCES ${DISPLAY_DIR}/*.c)

file(GLOB FOUNDATION_C_SOURCES
  ${FOUNDATION_DIR}/*.c)

file(GLOB_RECURSE MODULE_C_SOURCES
  ${MODULE_DIR}/*.c
  ${MODULE_DIR}/*/*.c
)

# Combine all app source files
set(APP_ALL_SOURCES 
    ${APP_VIDEO_C_SOURCES}
    ${APP_OPTFLOW_C_SOURCES}
    ${APP_DISPLAY_C_SOURCES}
    ${FOUNDATION_C_SOURCES}
    ${MODULE_C_SOURCES}
)

idf_component_register(
    SRCS "main.c" ${APP_ALL_SOURCES}
    INCLUDE_DIRS "." ${VIDEO_DIR} ${OPTFLOW_DIR} ${DISPLAY_DIR} ${FOUNDATION_DIR} ${MODULE_DIR} ${ROBOTKIT_DIR}
)

add_library(robotkit STATIC IMPORTED)
set_property(TARGET robotkit PROPERTY IMPORTED_LOCATION "${ROBOTKIT_DIR}/librobotkit-esp32p4.a")
target_link_libraries(${COMPONENT_LIB} PUBLIC robotkit)

add_library(optflow STATIC IMPORTED)
set_property(TARGET optflow PROPERTY IMPORTED_LOCATION "${OPTFLOW_DIR}/liboptflow-esp32p4.a")
target_link_libraries(${COMPONENT_LIB} PUBLIC optflow)