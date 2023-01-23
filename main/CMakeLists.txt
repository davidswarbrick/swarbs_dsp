set(SOURCES dsp.cpp WM8978.cpp reverb.cpp)
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "")
