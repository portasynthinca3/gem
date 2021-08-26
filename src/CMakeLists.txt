FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
idf_component_register(SRCS ${app_sources})
target_add_binary_data(${COMPONENT_TARGET} "bios.bin" BINARY)