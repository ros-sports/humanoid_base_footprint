# Build test_base_footprint
ament_add_gtest(test_base_footprint
    test_base_footprint.cpp)

target_link_libraries(test_base_footprint
  base_footprint_node
)