file(GLOB_RECURSE SOURCES "./*.c")

idf_component_register(
    SRCS
        ${SOURCES}
    INCLUDE_DIRS 
        "${OPEN_HOVER_ROOT_DIR}"
        "${OPEN_HOVER_ROOT_DIR}/src"
        "${OPEN_HOVER_ROOT_DIR}/src/oh_core"
        "${OPEN_HOVER_ROOT_DIR}/src/oh_example"
)
