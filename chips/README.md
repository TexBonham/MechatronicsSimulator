Chip definitions: platform description, SVD and pin-map template, one folder per chip.

Each folder holds `chip.yaml`, validated against `schemas/chip.json` (model `mechsim/design/chip.py`): name, family,
`status` (placeholder or verified), `platform_description` and `svd` (a file in the folder, or `placeholder`), and the
pin map (each pin with the peripheral modes it supports). A design's pin-binding table may use only pins and modes its
chip lists (DES-029).

`stm32_placeholder/` stands in for the gantry's unselected STM32 part. A real part replaces it by dropping in its SVD,
platform description and pin map and setting `status: verified`.
