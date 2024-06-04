project ("AISD-GRAPH")

add_executable (AISD-GRAPH "src/main.cpp"  "algorithm/algorithm.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET AISD-GRAPH PROPERTY CXX_STANDARD 20)
endif()