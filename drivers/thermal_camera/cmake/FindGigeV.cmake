# the installation script place
set(_gigev_sdk_conf "/etc/ld.so.conf.d/td_genicam_v3_0.conf")

if (EXISTS ${_gigev_sdk_conf})

    ###### --------------------------------------------------------------------
    # ROOT
    ######

    # get first line in td_genicam_v3_0.conf. then get the
    # parent direcotry of the first path which suppose to be the location of
    # the installed GENICAM SDK
    execute_process(
            COMMAND bash -c "head -n 1 /etc/ld.so.conf.d/td_genicam_v3_0.conf"
            OUTPUT_VARIABLE GENICAM_INSTALL_PATH
            #ENCODING UTF8
    )

    string(STRIP ${GENICAM_INSTALL_PATH} GENICAM_INSTALL_PATH)

    ######### -----------------------------------------------------------------
    # INCLUDE
    #########

    set(gigev_sdk_INCLUDE_DIRS
            ${GENICAM_INSTALL_PATH}/../../library/CPP/include
            /opt/dalsa/GigeV/include)
    set(gigev_sdk_INCLUDES ${gigev_sdk_INCLUDE_DIRS})

    ###### --------------------------------------------------------------------
    # LIBS
    ######

    if (EXISTS ${GENICAM_INSTALL_PATH}/libGCBase_gcc421_v3_0.so)
        set(GigeV_SDK_Build "Linux64_x64")
    elseif (EXISTS ${GENICAM_INSTALL_PATH}/libGCBase_gcc54_v3_0.so)
        set(GigeV_SDK_Build "Linux64_ARM")
    else ()
        message(FATAL_ERROR "GenICam not found. Please reinstall GigeV SDK from FLIR")
    endif ()

    if ("${GigeV_SDK_Build}" STREQUAL "Linux64_x64")

        set(gigev_sdk_LIBS
                ## GigeV SDK
                /usr/local/lib/libGevApi.so
                ## GenICam
                ${GENICAM_INSTALL_PATH}/libGCBase_gcc421_v3_0.so
                )
    elseif ("${GigeV_SDK_Build}" STREQUAL "Linux64_ARM")
        set(gigev_sdk_LIBS
                ## GigeV SDK
                /usr/local/lib/libGevApi.so
                ## GenICam
                ${GENICAM_INSTALL_PATH}/libGCBase_gcc54_v3_0.so
                ${GENICAM_INSTALL_PATH}/libGenApi_gcc54_v3_0.so
                )
    endif ()

    set(gigev_sdk_LIBRARIES ${gigev_sdk_LIBS})

    ####### -------------------------------------------------------------------
    # FOUND
    #######

    set(gigev_sdk_FOUND true)

else ()
    message(FATAL_ERROR "GigeV SDK is not installed. Please install GigeV SDK")
endif ()
