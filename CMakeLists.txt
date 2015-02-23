CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT TRUE)
SET(CMAKE_ECLIPSE_GENERATE_LINKED_RESOURCES FALSE)

PROJECT(RaceLogic)

SET(RACELOGIC_VERSION_MAJOR 1)
SET(RACELOGIC_VERSION_MINOR 0)
SET(RACELOGIC_VERSION_BUILD 0)

# Show relative paths options
OPTION(CMAKE_USE_RELATIVE_PATHS "If true, cmake will use relative paths in makefiles and projects." ON)

#Solution
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

###############################################################################
# Compiler Options
################################################################################
IF(WIN32)
	# Make sure we use minimal windows.h library without min max macros
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"WIN32_LEAN_AND_MEAN\"")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D \"NOMINMAX\"")
	
    IF(MSVC80)
		# Detech 64-bit portability Issues
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wp64")
    ENDIF(MSVC80)
	
	OPTION(MULTI_PROCESSOR_COMPILATION "Use multiple processors when compiling" ON)
	IF(MULTI_PROCESSOR_COMPILATION)
		# Set multi processor build
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	ENDIF(MULTI_PROCESSOR_COMPILATION)
	# Set warnings to level 4 
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
	IF (WARNINGS_AS_ERRORS)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
	ENDIF (WARNINGS_AS_ERRORS)
ENDIF(WIN32)

IF(UNIX)
	IF(CMAKE_COMPILER_IS_GNUCC)
		ADD_DEFINITIONS(-Wall -ansi -pedantic -Wextra)
		IF (WARNINGS_AS_ERRORS)
			ADD_DEFINITIONS(-Werror)		
		ENDIF(WARNINGS_AS_ERRORS)
	ENDIF(CMAKE_COMPILER_IS_GNUCC)
ENDIF(UNIX)

###############################################################################
# Compile subdirectory
################################################################################
ADD_SUBDIRECTORY(src)