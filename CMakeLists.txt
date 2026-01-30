cmake_minimum_required(VERSION 3.10)
project(mars_ode_physics VERSION 2.0.0 DESCRIPTION "This library is for the ode physics.")

find_package(Rock)

include(FindPkgConfig)

find_package(lib_manager)
lib_defaults()
define_module_info()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/cmake")


MACRO(CMAKE_USE_FULL_RPATH install_rpath)
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    SET(CMAKE_INSTALL_RPATH ${install_rpath})
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
ENDMACRO(CMAKE_USE_FULL_RPATH)
CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib")

#Get linker and compiler flags from pkg-config
pkg_check_modules(PKGCONFIG REQUIRED
       ode
       lib_manager
       data_broker
       #cfg_manager
       mars_interfaces
       mars_utils
       configmaps
       #envire_core
       data_broker
)


include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

#add_definitions(-DODE11=1 -DdDOUBLE)
#add_definitions(-DFORWARD_DECL_ONLY=1)

foreach(DIR ${CFG_MANAGER_INCLUDE_DIRS})
    set(ADD_INCLUDES "${ADD_INCLUDES} -I${DIR}")
endforeach(DIR)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src )

set(SOURCES_H
       src/WorldPhysicsLoader.hpp
       src/WorldPhysics.hpp
)
set(SOURCES_OBJECT_H
       src/objects/Box.hpp
       src/objects/Capsule.hpp
       src/objects/Cylinder.hpp
       src/objects/Frame.hpp
       src/objects/Inertial.hpp
       src/objects/Object.hpp
       src/objects/ObjectFactory.hpp
)
set(SOURCES_JOINT_H
       src/joints/JointFactory.hpp
       src/joints/Joint.hpp
       src/joints/HingeJoint.hpp
       src/joints/FixedJoint.hpp
       src/joints/Hinge2Joint.hpp
       src/joints/SliderJoint.hpp
       src/joints/UniversalJoint.hpp
)

set(TARGET_SRC
       src/WorldPhysicsLoader.cpp
       src/WorldPhysics.cpp
       src/objects/Frame.cpp
       src/objects/Object.cpp
       src/objects/ObjectFactory.cpp
       src/objects/Box.cpp
       src/objects/Inertial.cpp
       src/joints/JointFactory.cpp
       src/joints/Joint.cpp
       src/joints/HingeJoint.cpp
       src/joints/FixedJoint.cpp
       src/joints/Hinge2Joint.cpp
       src/joints/SliderJoint.cpp
       src/joints/UniversalJoint.cpp
)
add_definitions(-DSCHEMA_PATH=\"${CMAKE_INSTALL_PREFIX}/share/mars_ode_physics/schema\")
#cmake variables
#configure_file(${CMAKE_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h @ONLY)
include_directories("${CMAKE_BINARY_DIR}")
add_library(${PROJECT_NAME} SHARED ${TARGET_SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set(_INSTALL_DESTINATIONS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION lib
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
            ${PKGCONFIG_LIBRARIES}
            ${WIN_LIBS}
            pthread
)

add_executable(reproducable_test src/logging.cpp test/reproducable_test.cpp)
target_compile_features(reproducable_test PUBLIC cxx_std_17)
TARGET_LINK_LIBRARIES(reproducable_test
            ${PKGCONFIG_LIBRARIES}
            ${WIN_LIBS}
            pthread
)
target_compile_definitions(reproducable_test PRIVATE "-DTEST=1")
target_link_libraries(reproducable_test ${PROJECT_NAME})
configure_file(scripts/reproducable_test.sh ${CMAKE_BINARY_DIR} COPYONLY)
configure_file(scripts/reproducable_plot.py ${CMAKE_BINARY_DIR} COPYONLY)

if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)

set(_INSTALL_DESTINATIONS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION lib
)

# Install the library
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${SOURCES_H} DESTINATION include/${PROJECT_NAME})
install(FILES ${SOURCES_OBJECT_H} DESTINATION include/${PROJECT_NAME}/objects)
install(FILES ${SOURCES_JOINT_H} DESTINATION include/${PROJECT_NAME}/joints)
install(DIRECTORY configuration/schema/ DESTINATION share/mars_ode_physics/schema)
configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

# documentation
configure_file(${CMAKE_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_BINARY_DIR}/doc/Doxyfile @ONLY)
add_custom_target(doc
  COMMAND doxygen ${CMAKE_BINARY_DIR}/doc/Doxyfile
)

