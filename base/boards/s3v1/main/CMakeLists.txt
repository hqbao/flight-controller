set(FOUNDATION_DIR ../../../foundation)
set(MODULE_DIR ../../../../modules)
set(ROBOTKIT_DIR ../../../../libs/robotkit)

file(GLOB FOUNDATION_C_SOURCES ${FOUNDATION_DIR}/*.c)

file(GLOB_RECURSE MODULE_C_SOURCES
  ${MODULE_DIR}/*.c
  ${MODULE_DIR}/*/*.c
)

idf_component_register(
  SRCS 
  "main.c" 
  ${FOUNDATION_C_SOURCES}
  ${MODULE_C_SOURCES}
  INCLUDE_DIRS "." ${FOUNDATION_DIR} ${MODULE_DIR} ${ROBOTKIT_DIR}
)

target_compile_definitions(${COMPONENT_TARGET} PUBLIC "ESP32")

add_library(robotkit STATIC IMPORTED)
set_property(TARGET robotkit PROPERTY IMPORTED_LOCATION "../../../../Libs/robotkit/librobotkit-esp32s3.a")
target_link_libraries(${COMPONENT_LIB} PUBLIC robotkit)