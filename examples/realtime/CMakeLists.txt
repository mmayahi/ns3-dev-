if(${ENABLE_REALTIME})
  set(name realtime-udp-echo)
  set(source_files ${name}.cc)
  set(header_files)
  set(libraries_to_link ${libcsma} ${libinternet} ${libapplications})
  build_example(
    "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
  )
endif()
