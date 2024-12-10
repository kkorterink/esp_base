idf_component_register(SRCS "base_misc.c" "base_cmd.c" "base_main.c"
                    INCLUDE_DIRS ".")


# set_source_files_properties is used to set the compilation options to  a singel source file, see
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html

# set_source_files_properties(blink_example_main.c PROPERTIES COMPILE_FLAGS --coverage)

# --coverage NOT WORKING, THESE LINKER ERRORS:
# undefined reference to `__gcov_init
# undefined reference to `__gcov_exit
# undefined reference to `__gcov_merge_add
