# core/controller: contract

Namespace `mechsim::controller`. Target `mechsim_controller` (links `${CMAKE_DL_LIBS}`); tests `mechsim_controller_tests`,
`mechsim_controller_native_tests`. Public headers include only `pins/pins.hpp` (target `mechsim_pins`), `signals/pin_binding.hpp`,
`kernel/party.hpp` and `kernel/time.hpp`, never a kernel physics header (CTL-014, invariant 1); `mechsim_board.h` only privately in `src/`.

## Headers (`include/mechsim/controller/`)

| Header | Items |
|---|---|
| `controller_runtime.hpp` | `PinInputs`, `PinOutputs` (no heartbeat; header v2.2), `EventLogEntry`, size constants, `FirmwareImage{path, tick_hz}`, `ControllerRuntime` (`load`, `reset`, `tick`, `freeze`, `brownout`), `ControllerConfigError` (load), `ControllerError` (run), `NullControllerRuntime(Rate)` |
| `channel_map.hpp` | `PinSlot` (no direction/enable); `slot_capacity()`; `slot_accepts()`; `ChannelAssignment{signal, slot, channel}`; `ChannelMap`: `add()`, `find()`, `entries()`, `check_bindings()`, `assign_axis(axis, AxisChannels{dir_channel, enable_channel})`, `axis()`; `derive_axis_levels()` (CTL-006) |
| `controller_party.hpp` | `ControllerPartyConfig{name, rate, pin_inputs, rails, rail, latency_ticks}`; `ControllerParty(unique_ptr<ControllerRuntime>, config)`: role controller, publishes `<name>.pin_outputs`; with `rail`: reset on the first step, brownout on a voltage change; `schedule_freeze(start, end)`, `schedule_slowdown(start, end, factor)`; `record` writes the state code and the fault events |
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

Windows (`#ifdef _WIN32` branches; POSIX path unchanged): the host build runs through `CreateProcessW` (quoted command
line, stdout and stderr on one pipe, exit code from `GetExitCodeProcess`); loading is `LoadLibraryW` /
`GetProcAddress` / `FreeLibrary` on the same private copy; errors from `FormatMessageW`. The library name comes from
`native_library_name(app_target)`: `<app_target>_native.dll` on Windows. Firmware and mirror fixtures export through
`WINDOWS_EXPORT_ALL_SYMBOLS`. The MSVC environment (`cl`, Ninja) must be active wherever a host build runs.

## Tests

`tests/fixtures/mirror_app.cpp`: four shared fixtures (matching, version + 1, no version, 250 Hz). The
ctest fixture `mechsim_controller_native_reference_build` runs the hidden `[native-build]` case once (one real host
configure of `firmware/app/main_tick.c`); other native tests load its library from `native_work/reference/`.

## Scheduled faults and latency (`ControllerParty`, CTL-011, CTL-013, CTL-020, CTL-021, KER-021)

| Item | Behavior |
|---|---|
| Windows | Kernel ticks [start, end), scheduled before `start()` (later: `KernelConfigError`); end <= start or start < 0: `KernelConfigError` at the call; act at the party's first step at or after each bound (KER-023); a window holding none of the party's steps at its nominal rate: `KernelConfigError` at `start()`; with windows, a step period other than the nominal one faults the first step |
| Freeze | `freeze(true)` at the window's first step, `freeze(false)` at the first step at or after its end; the runtime is still ticked and returns held levels; physics and signals keep stepping. Overlapping freeze windows: `KernelConfigError` at `start()`; adjacent ones stay frozen |
| Slow-down | Integer factor >= 2 (else `KernelConfigError`); runtime ticks on the window's first step, then every factor-th step; between, `held_levels` of the last output. Overlapping slow-down windows: `KernelConfigError` at `start()`; a freeze inside a slow-down is allowed |
| Events | Each start and end: `RecordedEvent{t, "kernel", "fault", "<freeze or slowdown> start <party>[ factor N]"}` / `"... end <party>"`, on the first row after the step |
| Latency | `latency_ticks` control ticks through a FIFO; zeros until the first delayed output; events and state codes delay with the pins; transport latency, not flushed on freeze or brownout; 0 publishes the runtime's output unchanged |
| Brownout | Rail voltage from `signals.rails` (SIG-022); threshold is `NativeRuntimeConfig::brownout_threshold_v`, which the driver fills from the chip file's `brownout_threshold` (`chips/<chip>/chip.yaml`); default `placeholder_brownout_threshold_v` (2.7 V) |

`tests/test_controller_faults.cpp` (native test binary, mirror fixture) covers these through `CoSimKernel`.
`src/windows_text.hpp` (internal, Windows only): `detail::widen`, `narrow`, `error_text(DWORD)` shared by the host
build and the runtime.

## Per-tick use (KER-009)

Step 4: after sensors latch, `tick` runs once per control period; outputs then go to the signal layer. Physics and
signals keep stepping while frozen (KER-021). No Renode type crosses this interface (CTL-017).
