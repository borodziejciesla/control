################################
# GTest
################################
project(googletest-git NONE)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        8a6feab # release-1.10.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

################################
# Tests
################################
add_executable(${TEST_PROJECT_NAME}
    pid_test.cpp

    $<TARGET_OBJECTS:controller>
)

target_link_libraries(${TEST_PROJECT_NAME}
    gtest 
    gtest_main
        
    controller
)