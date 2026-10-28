cmake_minimum_required(VERSION 3.16)

project(SOEM
    DESCRIPTION "Simple Open EtherCAT Master"
    VERSION 1.4.0
    LANGUAGES C
)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

set(SOEM_INCLUDE_INSTALL_DIR include/soem)
set(SOEM_LIB_INSTALL_DIR lib)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(BUILD_TESTS TRUE)
else()
    message(STATUS "SOEM: not building tests when built as dependency")
    set(BUILD_TESTS FALSE)
endif()

if(UNIX AND NOT APPLE)
    set(OS "linux")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread rt)
else()
    message(FATAL_ERROR "This project is currently configured only for Linux.")
endif()

message(STATUS "OS is ${OS}")

add_library(soem STATIC
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatbase.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatcoe.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatconfig.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatdc.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercateoe.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatfoe.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatmain.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatprint.c
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatsoe.c
    ${CMAKE_CURRENT_LIST_DIR}/osal/linux/osal.c
    ${CMAKE_CURRENT_LIST_DIR}/oshw/linux/nicdrv.c
    ${CMAKE_CURRENT_LIST_DIR}/oshw/linux/oshw.c
)

target_link_libraries(soem PUBLIC ${OS_LIBS})
target_compile_options(soem PRIVATE -fPIC)

target_include_directories(soem PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal/linux>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/oshw/linux>
    $<INSTALL_INTERFACE:include>
)

message(STATUS "LIB_DIR: ${SOEM_LIB_INSTALL_DIR}")

install(TARGETS soem EXPORT soemConfig DESTINATION ${SOEM_LIB_INSTALL_DIR})
install(EXPORT soemConfig DESTINATION share/soem/cmake)

install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatbase.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatcoe.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatconfig.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatconfiglist.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatdc.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercateoe.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatfoe.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercat.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatmain.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatprint.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercatsoe.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/ethercattype.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/nicdrv.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/osal.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/oshw.h
    ${CMAKE_CURRENT_LIST_DIR}/soem/soem.h
    ${CMAKE_CURRENT_LIST_DIR}/osal/osal.h
    ${CMAKE_CURRENT_LIST_DIR}/osal/linux/osal_defs.h
    ${CMAKE_CURRENT_LIST_DIR}/oshw/linux/nicdrv.h
    ${CMAKE_CURRENT_LIST_DIR}/oshw/linux/oshw.h
    DESTINATION ${SOEM_INCLUDE_INSTALL_DIR}
)

# cmake_minimum_required(VERSION 3.9)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
# project(SOEM
#     DESCRIPTION "Simple Open EtherCAT Master"
#     VERSION 1.4.0
#     LANGUAGES C)

# if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#   # Default to installing in SOEM source directory
#   set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)
# endif()

# set(SOEM_INCLUDE_INSTALL_DIR include/soem)
# set(SOEM_LIB_INSTALL_DIR lib)

# if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
#   set(BUILD_TESTS TRUE)
# else()
#   message(STATUS "SOEM: not building tests when built as dependency")
#   set(BUILD_TESTS FALSE)
# endif()

# if(WIN32)
#   set(OS "win32")
#   include_directories(oshw/win32/wpcap/Include)
#   if(CMAKE_SIZEOF_VOID_P EQUAL 8)
#     link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw/win32/wpcap/Lib/x64)
#   elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
#     link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw/win32/wpcap/Lib)
#   endif()
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
#   set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
# elseif(UNIX AND NOT APPLE)
#   set(OS "linux")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
#   set(OS_LIBS pthread rt)
# elseif(APPLE)
#   # This must come *before* linux or MacOSX will identify as Unix.
#   set(OS "macosx")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
#   set(OS_LIBS pthread pcap)
# elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
#   set(OS "rtk")
#   message(STATUS "ARCH is ${ARCH}")
#   message(STATUS "BSP is ${BSP}")
#   include_directories(oshw/${OS}/${ARCH})
#   file(GLOB OSHW_EXTRA_SOURCES oshw/${OS}/${ARCH}/*.c)
#   set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
#   set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
# elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
#   message(STATUS "Building for RTEMS")
#   set(OS "rtems")
#   set(SOEM_LIB_INSTALL_DIR ${LIB_DIR})
#   set(BUILD_TESTS FALSE)
# endif()

# message(STATUS "OS is ${OS}")

# file(GLOB SOEM_SOURCES soem/*.c)
# file(GLOB OSAL_SOURCES osal/${OS}/*.c)
# file(GLOB OSHW_SOURCES oshw/${OS}/*.c)

# file(GLOB SOEM_HEADERS soem/*.h)
# file(GLOB OSAL_HEADERS osal/osal.h osal/${OS}/*.h)
# file(GLOB OSHW_HEADERS oshw/${OS}/*.h)

# add_library(soem STATIC
#   ${SOEM_SOURCES}
#   ${OSAL_SOURCES}
#   ${OSHW_SOURCES}
#   ${OSHW_EXTRA_SOURCES})
# target_link_libraries(soem ${OS_LIBS})
# target_compile_options(soem PRIVATE -fPIC)

# target_include_directories(soem PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
#   $<INSTALL_INTERFACE:include>
# )

# target_include_directories(soem PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal>
# )

# target_include_directories(soem PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal/${OS}>
# )

# target_include_directories(soem
#   PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/oshw/${OS}>
# )

# message(STATUS "LIB_DIR: ${SOEM_LIB_INSTALL_DIR}")

# install(TARGETS soem EXPORT soemConfig DESTINATION ${SOEM_LIB_INSTALL_DIR})

# install(EXPORT soemConfig DESTINATION share/soem/cmake)

# install(FILES
#   ${SOEM_HEADERS}
#   ${OSAL_HEADERS}
#   ${OSHW_HEADERS}
#   DESTINATION ${SOEM_INCLUDE_INSTALL_DIR})

# add_subdirectory(test/linux/firm_update)
# add_subdirectory(test/linux/slaveinfo)

# if(BUILD_TESTS)
#   add_subdirectory(test/simple_ng)
#   add_subdirectory(test/linux/slaveinfo)
#   add_subdirectory(test/linux/eepromtool)
#   add_subdirectory(test/linux/simple_test)
# endif()


