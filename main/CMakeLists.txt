FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/../*.*)

idf_component_register(SRCS ${app_sources}
                    INCLUDE_DIRS "../lib")