set(name dhcp-example)
set(source_files ${name}.cc)
set(header_files)
set(libraries_to_link ${libinternet} ${libinternet-apps} ${libcsma}
                      ${libpoint-to-point} ${libapplications}
)
build_lib_example(
  "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
)

set(name traceroute-example)
set(source_files ${name}.cc)
set(header_files)
set(libraries_to_link
    ${libaodv}
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libpoint-to-point}
    ${libwifi}
    ${libinternet-apps}
)
build_lib_example(
  "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
)
