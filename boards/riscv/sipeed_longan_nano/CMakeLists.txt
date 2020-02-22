# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(empty.c)
zephyr_library_sources(main.c)
zephyr_library_sources(pinmux.c)
zephyr_library_sources(systick.c)

set_property(GLOBAL PROPERTY CSTD "gnu11")
