set(name olsr-hna)
set(source_files ${name}.cc)
set(header_files)
set(libraries_to_link ${libcore} ${libmobility} ${libwifi} ${libcsma}
                      ${libolsr}
)
build_lib_example(
  "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
)

set(name simple-point-to-point-olsr)
set(source_files ${name}.cc)
set(header_files)
set(libraries_to_link ${libpoint-to-point} ${libinternet} ${libolsr}
                      ${libapplications} ${libwifi}
)
build_lib_example(
  "${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
)
