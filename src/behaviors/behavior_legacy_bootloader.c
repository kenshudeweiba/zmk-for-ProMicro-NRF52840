/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: MIT
 *
 * Legacy bootloader entry for Adafruit/Daxiong nRF52 bootloaders.
 *
 * ZMK's stock &bootloader uses retention bootmode on recent Zephyr, but these
 * older bootloaders only look for DFU_MAGIC_UF2_RESET (0x57) in GPREGRET.
 * This behavior writes that value directly, then resets the Cortex-M core
 * without going through sys_reboot(), so the magic value is preserved.
 */

#define DT_DRV_COMPAT zmk_behavior_legacy_bootloader

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include <hal/nrf_power.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define LEGACY_BOOT_DFU_MAGIC_UF2_RESET 0x57u

/* Cortex-M4 AIRCR: key 0x05FA + SYSRESETREQ (bit 28). */
#define LEGACY_BOOT_AIRCR_ADDR 0xE000ED0Cu
#define LEGACY_BOOT_AIRCR_KEY 0x05FA0000u
#define LEGACY_BOOT_AIRCR_SYSRESETREQ (1u << 28)

static int on_legacy_boot_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);

    nrf_power_gpregret_set(NRF_POWER, 0u, LEGACY_BOOT_DFU_MAGIC_UF2_RESET);

    /* Make the retention register write visible before the reset. */
    __asm__ volatile("dsb" ::: "memory");

    *((volatile uint32_t *)LEGACY_BOOT_AIRCR_ADDR) =
        LEGACY_BOOT_AIRCR_KEY | LEGACY_BOOT_AIRCR_SYSRESETREQ;

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_legacy_boot_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api legacy_boot_driver_api = {
    .binding_pressed = on_legacy_boot_binding_pressed,
    .binding_released = on_legacy_boot_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &legacy_boot_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
