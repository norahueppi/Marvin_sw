idf_component_register(SRCS "bit.c" "decoder.c" "fixed.c" "frame.c" "huffman.c" "layer12.c" "layer3.c" "stream.c" "synth.c" "timer.c" "version.c"
                  INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_TARGET} PRIVATE "-DFPM_DEFAULT" "-DOPT_ACCURACY" "-DNDEBUG" "-DSIZEOF_INT=4" "-DSIZEOF_LONG=4" "-DSIZEOF_LONG_LONG=8" "-Wno-error=unused-label" "-Wno-error=return-type" "-Wno-error=missing-braces" "-Wno-error=pointer-sign" "-Wno-error=parentheses" "-Wno-stringop-overflow")