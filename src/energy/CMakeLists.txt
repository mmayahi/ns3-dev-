set(name energy)

set(source_files
    helper/basic-energy-harvester-helper.cc
    helper/basic-energy-source-helper.cc
    helper/energy-harvester-container.cc
    helper/energy-harvester-helper.cc
    helper/energy-model-helper.cc
    helper/energy-source-container.cc
    helper/li-ion-energy-source-helper.cc
    helper/rv-battery-model-helper.cc
    model/basic-energy-harvester.cc
    model/basic-energy-source.cc
    model/device-energy-model-container.cc
    model/device-energy-model.cc
    model/energy-harvester.cc
    model/energy-source.cc
    model/li-ion-energy-source.cc
    model/rv-battery-model.cc
    model/simple-device-energy-model.cc
)

set(header_files
    helper/basic-energy-harvester-helper.h
    helper/basic-energy-source-helper.h
    helper/energy-harvester-container.h
    helper/energy-harvester-helper.h
    helper/energy-model-helper.h
    helper/energy-source-container.h
    helper/li-ion-energy-source-helper.h
    helper/rv-battery-model-helper.h
    model/basic-energy-harvester.h
    model/basic-energy-source.h
    model/device-energy-model-container.h
    model/device-energy-model.h
    model/energy-harvester.h
    model/energy-source.h
    model/li-ion-energy-source.h
    model/rv-battery-model.h
    model/simple-device-energy-model.h
)

set(libraries_to_link ${libnetwork})

set(test_sources test/basic-energy-harvester-test.cc
                 test/li-ion-energy-source-test.cc
)

build_lib("${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
          "${test_sources}"
)
