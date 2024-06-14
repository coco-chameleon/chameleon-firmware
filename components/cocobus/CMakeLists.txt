
idf_component_register(
    SRCS
        "src/cocobus.c"
        "src/interrupt.S"
        "src/timer.c"
    INCLUDE_DIRS "include"
    LDFRAGMENTS "placement.lf"
    PRIV_REQUIRES driver soc xtensa
)

target_link_libraries(${COMPONENT_TARGET} "-u ld_include_cocobus_interrupt")
