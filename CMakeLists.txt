cmake_minimum_required(VERSION 2.6)


INCLUDE(FindPkgConfig)

pkg_check_modules(IO REQUIRED "iodrivers_base")
INCLUDE_DIRECTORIES(${IO_INCLUDE_DIRS})
LINK_DIRECTORIES(${IO_LIBRARY_DIRS})

add_executable(lowlevel_led
	lowlevel_processor.cpp
	main-led.cpp
)

add_executable(lowlevel_laser
	lowlevel_processor.cpp
	main-laser.cpp
)

add_executable(lowlevel_reset
	lowlevel_processor.cpp
	main-reset.cpp
)

add_executable(lowlevel_lamps
	lowlevel_processor.cpp
	main-lamps.cpp
)

add_executable(lowlevel_test
	lowlevel_processor.cpp
	main.cpp
)

add_library(lowlevelDriver SHARED
	lowlevel_processor.cpp
)

TARGET_LINK_LIBRARIES(lowlevelDriver 
	${IO_LIBRARIES}
)

TARGET_LINK_LIBRARIES(lowlevel_lamps
	${IO_LIBRARIES}
)

TARGET_LINK_LIBRARIES(lowlevel_test 
	${IO_LIBRARIES}
)

TARGET_LINK_LIBRARIES(lowlevel_reset
	${IO_LIBRARIES}
)
TARGET_LINK_LIBRARIES(lowlevel_led
	${IO_LIBRARIES}
)

TARGET_LINK_LIBRARIES(lowlevel_laser
	${IO_LIBRARIES}
)

INSTALL(TARGETS lowlevelDriver 
	LIBRARY DESTINATION lib
)


INSTALL(FILES lowlevel_processor.h DESTINATION include)
	
CONFIGURE_FILE(lowlevelDriver.pc.in lowlevelDriver.pc @ONLY)

INSTALL(FILES 
	${CMAKE_BINARY_DIR}/lowlevelDriver.pc 
	DESTINATION lib/pkgconfig
)

