cmake_minimum_required(VERSION 3.5)
project(go2_hal_test)

find_package(go2hal QUIET)

add_executable(test_go2_hal
	test_go2_hal.cpp
)

target_link_libraries(test_go2_hal
	# PRIVATE
	PUBLIC
		go2hal
)

set_property(
  TARGET test_go2_hal
  PROPERTY
    CXX_STANDARD 17
)

# TODO: Implement the test for python
