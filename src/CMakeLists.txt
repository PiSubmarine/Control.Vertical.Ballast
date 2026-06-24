include("${PISUBMARINE_BUILD_SYSTEM}")

message("Adding library: ${PISUBMARINE_MODULE_NAME}")
add_library(${PISUBMARINE_MODULE_NAME}
        "../public/PiSubmarine/Control/Vertical/Ballast/Controller.h"
        "PiSubmarine/Control/Vertical/Ballast/Controller.cpp")
target_include_directories(${PISUBMARINE_MODULE_NAME} PUBLIC "../public")
PiSubmarineInitTarget(${PISUBMARINE_MODULE_NAME})

PiSubmarineAddDependency("https://github.com/PiSubmarine/Ballast.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PUBLIC PiSubmarine.Ballast.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Control.Vertical.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PUBLIC PiSubmarine.Control.Vertical.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Depth.Telemetry.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PUBLIC PiSubmarine.Depth.Telemetry.Api)

PiSubmarineAddDependency("https://github.com/PiSubmarine/Time.Tickable.Api.git" "")
target_link_libraries(${PISUBMARINE_MODULE_NAME} PUBLIC PiSubmarine.Time.Tickable.Api)
