set(name udp-client-server)
set(source_files ${name}.cc)
set(header_files)
set(libraries_to_link ${libcsma} ${libinternet} ${libapplications})
build_example(
  "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
)

set(name udp-trace-client-server)
set(source_files ${name}.cc)
set(header_files)
set(libraries_to_link ${libcsma} ${libinternet} ${libapplications})
build_example(
  "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
)
