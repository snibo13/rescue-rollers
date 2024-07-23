set(INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../include/rr_custom_messages")

file(GLOB_RECURSE EXTRA_INCLUDES ${INCLUDE_DIRS}/*)
set(ALL_INCLUDE_DIRS ${INCLUDE_DIRS})
foreach(dir ${EXTRA_INCLUDES})
    if(IS_DIRECTORY ${dir})
        list(APPEND ALL_INCLUDE_DIRS ${dir})
    endif()
endforeach()

idf_component_register(SRCS  main.cpp
                    INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../include/rr_custom_messages" 
                    PRIV_REQUIRES driver micro_ros_espidf_component arduino
                    )