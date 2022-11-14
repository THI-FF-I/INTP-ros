if(${CURSES_FOUND})
    add_executable(jps_maze_curses src/jps_maze_curses.cpp)
    target_include_directories(jps_maze_curses PRIVATE
            include
            ${CURSES_INCLUDE_DIRS})
    target_link_libraries(jps_maze_curses PRIVATE
            ${CURSES_LIBRARIES})
    install(TARGETS
            jps_maze_curses
            DESTINATION share/${PROJECT_NAME})
else()
    message(WARNING "Could not find curses, skipping jps_maze_curses")
endif()