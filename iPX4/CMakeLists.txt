#--------------------------------------------------------
#    NAME: Dongbin Kim
#    ORGN: University of Hartford
#    (based on pMavlink by  David Battle @ Mission Systems Pty Ltd
#--------------------------------------------------------
cmake_minimum_required(VERSION 3.5)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{HOME}/bin)
set(CMAKE_INSTALL_PREFIX /usr/local/bin)

project(moos-ipx4)

#===============================================================================
# FINDING MOOS' HEADERS AND LIBRARIES...
#===============================================================================

find_package(MOOS 10.0)

set(SRC
    main.cpp
    PX4.cpp
    PX4_Info.cpp)

add_executable(iPX4 ${SRC})
message("MOOS_LIBRARIES:   " ${MOOS_DIR})


target_include_directories(iPX4 PUBLIC
    ./
    ${MOOS_DIR}/../../../include/ivp
    $ENV{HOME}/mavlink/include/common
    ${MOOS_INCLUDE_DIRS})

target_link_libraries(iPX4
   ${MOOS_DIR}/../../../lib/libmbutil.a
   ${MOOS_LIBRARIES}
   boost_system)

install(TARGETS iPX4
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
