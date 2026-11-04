# core/controller: contract

Namespace `mechsim::controller`. Target `mechsim_controller` (links `${CMAKE_DL_LIBS}`); tests `mechsim_controller_tests`,
`mechsim_controller_native_tests`. Public headers include only `signals/power_rail.hpp`, `signals/pin_binding.hpp` and
`kernel/time.hpp`, never a kernel physics header (CTL-014, invariant 1); `mechsim_board.h` only privately in `src/`.

## Headers (`include/mechsim/controller/`)

| Header | Items |
|---|---|
| `controller_runtime.hpp` | `PinInputs`, `PinOutputs` (no heartbeat; header v2.2), `EventLogEntry`, size constants, `FirmwareImage{path, tick_hz}`, `ControllerRuntime` (`load`, `reset`, `tick`, `freeze`, `brownout`), `ControllerConfigError` (load), `ControllerError` (run), `NullControllerRuntime(Rate)` |
| `channel_map.hpp` | `PinSlot` (no direction/enable); `slot_capacity()`; `slot_accepts()`; `ChannelAssignment{signal, slot, channel}`; `ChannelMap`: `add()`, `find()`, `entries()`, `check_bindings()`, `assign_axis(axis, AxisChannels{dir_channel, enable_channel})`, `axis()`; `derive_axis_levels()` (CTL-006) |
| `native_controller_runtime.hpp` | `board_interface_version()`; `placeholder_brownout_threshold_v`; `HostBuildRequest`, `build_host_firmware()` -> `{library, log}`; `NativeRuntimeConfig{control_rate, work_dir, brownout_threshold_v, channels, app_target, cmake_command}`; `NativeControllerRuntime` + `channel_map()`, `library_path()`, `build_log()`, `ticks_run()`, `inits()`, `frozen()`, `held_in_reset()`, `supply_voltage()` |

## Native runtime (CTL-002 to CTL-007, CTL-011 to CTL-013, CTL-018)

| Step | Behavior |
|---|---|
| `load`, directory | `cmake --preset host -S <tree> -B <work_dir>/native-firmware/host -DMECHSIM_APP_TICK_HZ=<tick_hz> -DMECHSIM_FW_BUILD_TESTS=OFF`, `cmake --build ... --target <app_target>_native` -> `lib<app_target>_native.so` |
| `load`, file | Loads a library built that way (or a fixture with the four entry points) |
| `load`, checks | `tick_hz` equals the control rate before building; `mechsim_app_board_interface_version()` equals the runtime's header version; `mechsim_app_tick_hz()` equals the control rate. Failures: `ControllerConfigError` with tool output or `dlerror` |
| Isolation | Each load copies the library to `<work_dir>/native-controller/<pid>-<n>/` and `dlopen`s it `RTLD_NOW|RTLD_LOCAL` (RAII), so runtimes never share firmware statics |
| `reset(rail)` | Rail (0 V when off) at or above threshold (optional; unset throws): fresh copy, init; below: held |
| `brownout(v)` | Below threshold: held, outputs zero; recovery after a reset: fresh copy, init |
| `tick` | rx count > 64 throws; frozen: `held_levels()` (levels hold, buffers empty), no call; held: zero outputs; else copy field for field, call tick, derive direction/enable, tx count > 64 or event count > 16 throws `ControllerError` |

Polled (ADR 0011 C3): deltas and levels pass through unchanged; no threads, no interrupts.

## Tests

`tests/fixtures/mirror_app.cpp`: four shared fixtures (matching, version + 1, no version, 250 Hz). The
ctest fixture `mechsim_controller_native_reference_build` runs the hidden `[native-build]` case once (one real host
configure of `firmware/app/main_tick.c`); other native tests load its library from `native_work/reference/`.

## Per-tick use (KER-009)

Step 4: after sensors latch, `tick` runs once per control period; outputs then go to the signal layer. Physics and
signals keep stepping while frozen (KER-021). No Renode type crosses this interface (CTL-017).
