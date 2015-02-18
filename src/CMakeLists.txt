# Target name
SET(TARGET_LIBRARYNAME RaceLogic)
SET(TARGET_TARGETNAME_CPP RaceLogicCPPExample)
SET(TARGET_TARGETNAME_C RaceLogicCExample)

# Target source files
SET(TARGET_LIB_SRC
	racelogic.cpp
)

# Target headerfiles
SET(TARGET_LIB_H
	racelogic.h
)

#####################################################################
# Create library
#####################################################################

ADD_LIBRARY(${TARGET_LIBRARYNAME} ${TARGET_LIB_SRC} ${TARGET_LIB_H})

# Link to libraries needed
IF(MSVC)
	TARGET_LINK_LIBRARIES(${TARGET_LIBRARYNAME} ws2_32.lib)
ENDIF(MSVC)

#####################################################################
# Create example executable
#####################################################################

ADD_EXECUTABLE(${TARGET_TARGETNAME_CPP} main.cpp)
TARGET_LINK_LIBRARIES(${TARGET_TARGETNAME_CPP} ${TARGET_LIBRARYNAME})

ADD_EXECUTABLE(${TARGET_TARGETNAME_C} main.c)
TARGET_LINK_LIBRARIES(${TARGET_TARGETNAME_C} ${TARGET_LIBRARYNAME})

