project(cpp_wrapper LANGUAGES CXX)
add_library(${PROJECT_NAME} SHARED src/wrapper_api.cpp)
target_include_directories(${PROJECT_NAME} 
PRIVATE 
    ${CMAKE_SOURCE_DIR}/third_party/nlohmann_json
    include/)
target_link_libraries(${PROJECT_NAME} PRIVATE flexiv::flexiv_rdk)

if (MSVC)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LINK_FLAGS "/NOIMPLIB /NOEXP"
    )
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(LIB_NAME "flexiv_rdk.win64-vs2019")
else()
    set(LIB_NAME "flexiv_rdk.win64-vs2019")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME ${LIB_NAME}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${PROJECT_NAME}/lib 
        RUNTIME DESTINATION ${PROJECT_NAME}/bin
        LIBRARY DESTINATION ${PROJECT_NAME}/lib
)