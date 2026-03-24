add_executable(test test.c)
target_link_libraries(test soem)
install(TARGETS test DESTINATION bin)
