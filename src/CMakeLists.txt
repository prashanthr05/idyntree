# include test data before everything,
# to be sure that is available to test
# of all components
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests/data)
endif()

add_subdirectory(core)
add_subdirectory(model)
add_subdirectory(sensors)

if(IDYNTREE_USES_KDL)
    add_subdirectory(kdl)
    add_subdirectory(model_io)
    add_subdirectory(regressors)
    if(IDYNTREE_USES_YARP)
        add_subdirectory(yarp)
        add_subdirectory(estimation)
        if(IDYNTREE_USES_ICUB_MAIN)
            add_subdirectory(icub)
        endif()
    endif()
    add_subdirectory(high-level)
endif()

# compile simple command line tools
add_subdirectory(tools)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()