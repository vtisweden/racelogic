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

IF(MSVC)
	SET(CMAKE_DEBUG_POSTFIX "d")
ENDIF(MSVC)

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

#####################################################################
# Install library
#####################################################################

# Copy the library header files
INSTALL(FILES ${TARGET_LIB_H} DESTINATION include)
INSTALL(TARGETS ${TARGET_LIBRARYNAME}
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
)
INSTALL(FILES "${PROJECT_SOURCE_DIR}/README.md" DESTINATION . RENAME readme.txt)
INSTALL(FILES "${PROJECT_SOURCE_DIR}/LICENSE" DESTINATION . RENAME license.txt)

SET(CPACK_PACKAGE_NAME "${TARGET_LIBRARYNAME}")
SET(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
SET(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
SET(CPACK_PACKAGE_VERSION_MAJOR "${RACELOGIC_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${RACELOGIC_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${RACELOGIC_VERSION_BUILD}")

IF(WIN32)
SET(CPACK_GENERATOR "ZIP")
ELSEIF(UNIX)
SET(CPACK_GENERATOR "TGZ;TBZ2;TZ")
ENDIF()

# Pack files to package
INCLUDE(CPack)