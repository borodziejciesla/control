###########################################################
##################### Control Example #####################
###########################################################
find_package(Python3 COMPONENTS Interpreter Development)
if (${Python3_FOUND})
  include_directories(${Python3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

add_executable(${EXAMPLE_PROJECT_NAME}
  example.cpp
  matplotlibcpp.hpp
)

include_directories(${EXAMPLE_PROJECT_NAME}
  PUBLIC
  $<TARGET_PROPERTY:controller,INTERFACE_INCLUDE_DIRECTORIES>
  ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${EXAMPLE_PROJECT_NAME}
  controller
  ${Python3_LIBRARIES}
)