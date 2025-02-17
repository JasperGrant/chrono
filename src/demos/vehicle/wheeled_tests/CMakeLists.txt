#=============================================================================
# CMake configuration file for the ISO2631 demos.
# These example programs work with and without Irrlicht run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# List all ISO2631 demos

set(DEMOS
    demo_VEH_Test_ISO2631_Ride
    demo_VEH_Test_ISO2631_Shock
    demo_VEH_Test_Accel
    demo_VEH_Test_FMTV_Ride
)

if(HAVE_OPENCRG)
   if(ENABLE_MODULE_VSG)
      set(DEMOS ${DEMOS}
          demo_VEH_Test_FEDA_Ride
          demo_VEH_Test_Cornering
   )
   endif()
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()

if(ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "ChronoEngine_postprocess")
endif()

#--------------------------------------------------------------
# Create the executables

set(MODELS ../WheeledVehicleModels.h)

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp ${MODELS})
    source_group("" FILES ${PROGRAM}.cpp ${MODELS})

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
