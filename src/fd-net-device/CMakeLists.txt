set(name fd-net-device)

mark_as_advanced(ENABLE_THREADING)
set(ENABLE_THREADING ${HAVE_PTHREAD_H})

check_include_file_cxx(net/ethernet.h HAVE_NET_ETHERNET_H)
check_include_file_cxx(netpacket/packet.h HAVE_PACKET_H)
check_include_file_cxx(net/if.h HAVE_IF_NETS_H)
check_include_file_cxx(linux/if_tun.h HAVE_IF_TUN_H)
check_include_file_cxx(net/netmap_user.h HAVE_NETMAP_USER_H)
check_include_file_cxx(sys/ioctl.h HAVE_SYS_IOCTL_H)

include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(DPDK libdpdk)
endif()

mark_as_advanced(
  ENABLE_FDNETDEV ENABLE_DPDKDEVNET ENABLE_TAPNETDEV ENABLE_EMUNETDEV
  ENABLE_NETMAP_EMU
)
set(ENABLE_FDNETDEV False CACHE INTERNAL "")
set(ENABLE_DPDKDEVNET False CACHE INTERNAL "")
set(ENABLE_TAPNETDEV False CACHE INTERNAL "")
set(ENABLE_EMUNETDEV False CACHE INTERNAL "")
set(ENABLE_NETMAP_EMU False CACHE INTERNAL "")

if(${THREADS_ENABLED} AND HAVE_NET_ETHERNET_H)
  set(ENABLE_FDNETDEV True CACHE INTERNAL "")

  if(PKG_CONFIG_FOUND AND DPDK_FOUND)
    set(ENABLE_DPDKDEVNET True CACHE INTERNAL "")
  endif()

  if(HAVE_IF_NETS_H
     AND HAVE_IF_TUN_H
     AND HAVE_SYS_IOCTL_H
     AND ${ENABLE_TAP}
  )
    set(ENABLE_TAPNETDEV True CACHE INTERNAL "")
  endif()

  if(HAVE_IF_NETS_H
     AND HAVE_PACKET_H
     AND HAVE_SYS_IOCTL_H
     AND ${ENABLE_EMU}
  )
    set(ENABLE_EMUNETDEV True CACHE INTERNAL "")
    add_definitions(-DHAVE_PACKET_H)
  endif()

  if(HAVE_IF_NETS_H AND HAVE_NETMAP_USER_H AND HAVE_SYS_IOCTL_H)
    set(ENABLE_NETMAP_EMU True CACHE INTERNAL "")
    add_definitions(-DHAVE_NETMAP_USER_H)
  endif()
endif()

if(${ENABLE_FDNETDEV})
  set(fd-net-device_creators)

  if(${ENABLE_EMUNETDEV})
    set(emu_sources helper/emu-fd-net-device-helper.cc)
    set(emu_headers helper/emu-fd-net-device-helper.h)

    add_executable(
      raw-sock-creator helper/creator-utils.cc helper/encode-decode.cc
                       helper/raw-sock-creator.cc
    )
    add_definitions(
      -DRAW_SOCK_CREATOR="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/src/fd-net-device/ns${NS3_VER}-raw-sock-creator${build_profile_suffix}"
    )
    set_runtime_outputdirectory(
      raw-sock-creator ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/src/fd-net-device/ ""
    )

    list(APPEND fd-net-device_creators raw-sock-creator)
  endif()

  if(${ENABLE_TAPNETDEV})
    set(tap_sources helper/tap-fd-net-device-helper.cc)
    set(tap_headers helper/tap-fd-net-device-helper.h)

    add_executable(
      tap-device-creator helper/creator-utils.cc helper/encode-decode.cc
                         helper/tap-device-creator.cc
    )
    add_definitions(
      -DTAP_DEV_CREATOR="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/src/fd-net-device/ns${NS3_VER}-tap-device-creator${build_profile_suffix}"
    )
    set_runtime_outputdirectory(
      tap-device-creator ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/src/fd-net-device/
      ""
    )

    list(APPEND fd-net-device_creators tap-device-creator)
  endif()

  if(${ENABLE_NETMAP_EMU})
    set(netmap_sources helper/netmap-net-device-helper.cc
                       model/netmap-net-device.cc
    )
    set(netmap_headers helper/netmap-net-device-helper.h
                       model/netmap-net-device.h
    )

    add_executable(
      netmap-device-creator helper/creator-utils.cc helper/encode-decode.cc
                            helper/netmap-device-creator.cc
    )
    add_definitions(
      -DNETMAP_DEV_CREATOR="${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/src/fd-net-device/ns${NS3_VER}-netmap-device-creator${build_profile_suffix}"
    )
    set_runtime_outputdirectory(
      netmap-device-creator
      ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/src/fd-net-device/ ""
    )

    list(APPEND fd-net-device_creators netmap-device-creator)
  endif()

  if(${ENABLE_DPDKDEVNET})
    set(dpdk_sources model/dpdk-net-device.cc helper/dpdk-net-device-helper.cc)
    set(dpdk_headers model/dpdk-net-device.h helper/dpdk-net-device-helper.h)
    add_definitions(-DHAVE_DPDK_USER_H)
  endif()

  set(source_files
      ${tap_sources}
      ${emu_sources}
      ${netmap_sources}
      ${dpdk_sources}
      helper/creator-utils.cc
      helper/encode-decode.cc
      helper/fd-net-device-helper.cc
      model/fd-net-device.cc
  )

  set(header_files
      ${tap_headers} ${emu_headers} ${dpdk_headers}
      model/fd-net-device.h helper/fd-net-device-helper.h
  )

  set(libraries_to_link ${libnetwork} ${LIB_AS_NEEDED_PRE} ${DPDK_LIBRARIES}
                        ${LIB_AS_NEEDED_POST}
  )

  set(test_sources)

  build_lib("${name}" "${source_files}" "${header_files}"
            "${libraries_to_link}" "${test_sources}"
  )

  if(${ENABLE_DPDKDEVNET})
    target_include_directories(
      ${libfd-net-device-obj} PRIVATE ${DPDK_INCLUDE_DIRS}
    )
    target_compile_options(${libfd-net-device-obj} PRIVATE ${DPDK_CFLAGS})
  endif()

  list(LENGTH fd-net-device_creators num_creators)
  if(${num_creators} GREATER 0)
    add_dependencies(${libfd-net-device} ${fd-net-device_creators})
  endif()
endif()
