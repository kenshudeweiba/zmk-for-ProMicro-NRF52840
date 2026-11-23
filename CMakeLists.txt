if(TARGET app)
  target_sources(app PRIVATE src/behaviors/behavior_legacy_bootloader.c)
  target_sources(app PRIVATE src/features/eye_leds.c)
endif()
