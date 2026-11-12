Chip definitions: platform description, SVD and pin-map template, one folder per chip.

Each folder holds `chip.yaml`, validated against `schemas/chip.json` (model `mechsim/design/chip.py`): name, family,
`status` (placeholder or verified), `platform_description` and `svd` (a file in the folder, or `placeholder`), and the
pin map (each pin with the peripheral modes it supports), and the chip's electrical facts, all required: `core_voltage`,
`brownout_threshold` (strictly below the core voltage), `adc` (`resolution_bits` 1 to 32, `reference_voltage`) and
`counter_bits` (timer encoder counter width, 1 to 64). Voltages are unit strings such as `"3.3 V"`. A design's pin-binding table may use only pins and modes its
chip lists (DES-029).

`stm32_placeholder/` stands in for the gantry's unselected STM32 part; its electrical facts are placeholders, each
marked in the file, owed by the gantry team. A real part replaces it by dropping in its SVD,
platform description and pin map and setting `status: verified`. The gantry team selected the NUCLEO-G474RE
(STM32G474RE, 2026-11-11): `stm32_placeholder/` stands in for the STM32G474RE until the milestone 4 peripheral smoke
test; no chip file changes before then.
