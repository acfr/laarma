# the installation script place
set(_neuvition_sdk_conf "/opt/neuvition/lib/libneusdk.so")

if (EXISTS ${_neuvition_sdk_conf})
    set(neuvition_sdk_installation_root "/opt/neuvition")
    ######### -----------------------------------------------------------------
    # INCLUDE
    #########

    set(neuvition_sdk_INCLUDE_DIRS
            ${neuvition_sdk_installation_root}/include/)
    set(neuvition_sdk_INCLUDES ${neuvition_sdk_INCLUDE_DIRS})

    ###### --------------------------------------------------------------------
    # LIBS
    ######

    set(neuvition_sdk_LIBS
            ## Neuvition SDK
            ${neuvition_sdk_installation_root}/lib/libneusdk.so
            )

    set(neuvition_sdk_LIBRARIES ${neuvition_sdk_LIBS})

    ####### -------------------------------------------------------------------
    # FOUND
    #######

    set(neuvition_sdk_FOUND true)

else ()
    message(FATAL_ERROR "Neuvition SDK is not installed. Please install Neuvition SDK "
            "using the script install_sdk.sh")
endif ()
