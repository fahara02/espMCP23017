set(COMPONENT_SRCDIRS
    "src" "src/CORE" "src/COM"  "src/Util" 
)

set(COMPONENT_ADD_INCLUDEDIRS
  "src" "src/CORE" "src/COM"  "src/Util" 
)

set(COMPONENT_REQUIRES "ETL" "Logger")

register_component()

target_compile_definitions(${COMPONENT_TARGET} PUBLIC -DESP32)
target_compile_options(${COMPONENT_TARGET} PRIVATE -fno-rtti)

