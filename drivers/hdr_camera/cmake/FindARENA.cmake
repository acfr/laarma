# the installation script place
set(_arena_sdk_conf "/etc/ld.so.conf.d/Arena_SDK.conf")

if (EXISTS ${_arena_sdk_conf})

    ###### --------------------------------------------------------------------
    # ROOT
    ######

    # get first line in Arena_SDK.conf which is the lib64 path. then get the
    # parent direcotry of the first path which suppose to be the location of
    # the installed ArenaSDK
    execute_process(
            COMMAND bash -c "dirname $(head -n 1 \"/etc/ld.so.conf.d/Arena_SDK.conf\")"
            OUTPUT_VARIABLE arena_sdk_installation_root
            #ENCODING UTF8
    )

    string(STRIP ${arena_sdk_installation_root} arena_sdk_installation_root)

    ######### -----------------------------------------------------------------
    # INCLUDE
    #########

    set(arena_sdk_INCLUDE_DIRS
            ${arena_sdk_installation_root}/GenICam/library/CPP/include
            ${arena_sdk_installation_root}/include/)
    set(arena_sdk_INCLUDES ${arena_sdk_INCLUDE_DIRS})

    ###### --------------------------------------------------------------------
    # LIBS
    ######

    if (EXISTS ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so)
        set(ArenaSDK_Build "Linux64_x64_pre_54")
    elseif (EXISTS ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc54_v3_3_LUCID.so)
        set(ArenaSDK_Build "Linux64_x64_54")
    elseif (EXISTS ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGCBase_gcc54_v3_3_LUCID.so)
        set(ArenaSDK_Build "Linux64_ARM")

    else ()
        message(FATAL_ERROR "LUCID GenICam not found. Please reinstall ArenaSDK "
                "If having issues, contact: "
                "LUCID support team (support@thinklucid.com). ")
    endif ()

    if ("${ArenaSDK_Build}" STREQUAL "Linux64_x64_pre_54")

        set(arena_sdk_LIBS
                ## ArenaSDK
                ## release
                ${arena_sdk_installation_root}/lib64/libarena.so
                ${arena_sdk_installation_root}/lib64/libsave.so
                ${arena_sdk_installation_root}/lib64/libgentl.so
                ## GenICam
                ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so
                ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGenApi_gcc421_v3_0.so
                )
    elseif ("${ArenaSDK_Build}" STREQUAL "Linux64_x64_54")
        set(arena_sdk_LIBS
                ## ArenaSDK
                ## release
                ${arena_sdk_installation_root}/lib64/libarena.so
                ${arena_sdk_installation_root}/lib64/libsave.so
                ${arena_sdk_installation_root}/lib64/libgentl.so
                )
    elseif ("${ArenaSDK_Build}" STREQUAL "Linux64_ARM")
        set(arena_sdk_LIBS
                ## ArenaSDK
                ## release
                ${arena_sdk_installation_root}/lib/libarena.so
                ${arena_sdk_installation_root}/lib/libsave.so
                ${arena_sdk_installation_root}/lib/libgentl.so
                ## GenICam
                ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGCBase_gcc54_v3_3_LUCID.so
                ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGenApi_gcc54_v3_3_LUCID.so
                )
    endif ()

    set(arena_sdk_LIBRARIES ${arena_sdk_LIBS})
    set(arena_sdk_FOUND true)

else ()
    message(FATAL_ERROR "ArenaSDK is not installed. Please install ArenaSDK "
            "using the script provided by LUCID support "
            "team (support@thinklucid.com). ")
endif ()
