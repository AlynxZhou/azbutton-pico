configure_file("config.h.in" "config.h")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_executable("azbutton-pico" "main.c")

target_link_libraries(
  "azbutton-pico"
  PRIVATE
  "hardware_irq"
  "hardware_resets"
  "hardware_gpio"
  "pico_stdlib"
)
pico_add_extra_outputs("azbutton-pico")
