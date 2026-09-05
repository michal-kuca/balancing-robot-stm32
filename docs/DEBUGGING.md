# Debug Log — Self-Balancing Robot

A chronological record of every significant problem encountered during this build, its root cause, and the fix. Kept partly as engineering documentation, partly as proof that the robot did not work on the first try (it very much did not).

Hardware: STM32F103 (Nucleo-F103RB, later a clone board), MPU-6050 IMU, DRV8833 dual H-bridge, 2× geared DC motors with encoders, 2S 18650 battery pack.

---

## Part 1 — Hardware bring-up

### 1. Destroyed Nucleo board — loose battery wire touched the MCU
- **Symptom:** Smoke smell, STM32 chip too hot to touch, USB enumeration gone. Board completely dead.
- **Root cause:** A battery wire that was disconnected at one end brushed against the STM32's pins during rewiring. Battery voltage (far above the F103's ~4 V absolute maximum pin rating) went directly onto 3.3 V logic pins. The chip shorted internally and dumped the battery current as heat.
- **Fix:** Replacement board ordered. Before rebuilding, the breadboard was continuity-mapped with a multimeter to confirm no path existed from the battery rail to any logic pin except through the driver's VM input.
- **Lesson:** A live wire connected at only one end is the most dangerous thing on the bench. New rules adopted: (1) battery disconnected before *any* rewiring, no exceptions; (2) a wire is either fully connected or fully removed from the circuit; (3) a removable jumper/connector in the battery + line as a master disconnect. (The board's E5V/VIN external-power paths had also been damaged earlier, both reading ~2.2 V — the board was already wounded before the fatal incident.)

### 2. Power architecture revision
- **Symptom:** The buck converter + E5V power path caused repeated problems and was one adjustment mistake away from killing another board.
- **Decision:** Buck converter retired entirely. Motors powered directly from the 2S 18650 pack (7.4 V nominal); the Nucleo tethered via USB for all development and PID tuning. Untethered operation deferred to the very end via VIN (7–12 V input, onboard regulator — no external converter needed).
- **Lesson:** Don't debug a flaky control loop and a flaky power path at the same time. Sequence the risk: prove the control loop on rock-solid USB power first; untethering is then a ten-minute final step.

### 3. Motors dead — driver standby pin floating
- **Symptom:** Both motors completely unresponsive despite correct control signals; fault came and went between rebuilds.
- **Root cause:** The motor driver's standby/sleep pin was left floating instead of being tied high. A floating enable pin means the driver's outputs are (unpredictably) disabled.
- **Fix:** Standby pin wired to 3.3 V. Both motors immediately spun under static logic-level control (AIN1 high, AIN2 low), proving the full hardware chain end to end.
- **Lesson:** Enable/standby pins are the first thing to check when a driver ignores correct inputs. This same fault recurred multiple times during rewiring — it earns a permanent spot on the pre-flight checklist.

### 4. Intermittent faults — bad solder joints and Dupont jumpers
- **Symptom:** Connections that worked one session and failed the next; faults that moved when the harness was touched.
- **Root cause:** Cold/bad solder joints and worn Dupont jumper connections, especially on power lines.
- **Fix:** Solder braid used to clear clogged through-holes; power lines direct-soldered instead of running through header pins.
- **Lesson:** For anything carrying motor current, connectors are a liability. Solder the power path.

### 5. Clone board quirk — USB power path non-functional
- **Symptom:** SWD flashing failed with the board on USB only.
- **Root cause:** On the replacement (clone) board, the USB power path doesn't power the target properly — the MCU only runs with the battery connected.
- **Fix / workaround:** Established flashing ritual: battery connected, robot lying on its side (wheels free), then flash over SWD.
- **Lesson:** Clone boards save money and cost assumptions. Verify every power path on a new board before trusting it.

---

## Part 2 — Sensors and control

### 6. PID had no effect — hardcoded PWM override left in the loop
- **Symptom:** Tuning gains changed nothing; motors ran at a fixed speed regardless of tilt.
- **Root cause:** A leftover test block wrote a hardcoded PWM value to the compare registers *after* the PID output was applied, silently overriding the controller every cycle.
- **Fix:** Removed the override block.
- **Lesson:** Test scaffolding must be deleted, not commented "for later." One forgotten line invalidated an entire tuning session.

### 7. Wrong IMU axes and signs
- **Symptom:** Computed angle bore no resemblance to the physical tilt; filter output drifted or reacted to the wrong motion.
- **Root cause:** The accelerometer and gyro axis assignments (and their signs) didn't match the sensor's physical mounting orientation on the chassis.
- **Fix:** Corrected iteratively using live serial prints: tilt the robot a known direction, observe which raw axis responds and with what sign, fix the mapping, repeat.
- **Lesson:** Never assume the datasheet axes match your mounting. Calibrate the mapping empirically before trusting any fused output.

### 8. Complementary filter fighting itself — flipped gyro sign
- **Symptom:** The filtered angle was unstable/sluggish even though both raw sensors individually looked sane.
- **Root cause:** The gyro term entered the complementary filter with the wrong sign, so the gyro integration and the accelerometer correction pulled the estimate in opposite directions.
- **Fix:** Sign corrected; filter form verified as `angle = 0.98*(angle + gyro*dt) + 0.02*accel_angle` with the grouping intact.
- **Lesson:** In sensor fusion, a single sign error doesn't produce garbage — it produces something *almost* plausible, which is worse. Verify each input's sign independently before fusing.

### 9. Robot gave up past ~7° of tilt — duty overflow became "brake"
- **Symptom:** Correction worked for small tilts, then the wheels effectively stopped fighting once tilt exceeded roughly 7°.
- **Root cause:** The PID output mapped to a duty value that exceeded the timer period (99). Values above the period saturate the PWM at 100 %, which in the H-bridge input scheme in use turned the drive command into a brake/invalid state instead of maximum drive.
- **Fix:** Output clamped to the valid duty range before writing compare registers.
- **Lesson:** Every value written to a peripheral register needs an explicit range clamp. The hardware will not extrapolate your intent.

### 10. "Very weird" asymmetric behavior — motor A never reversed
- **Symptom:** Robot corrected properly in one direction and misbehaved in the other; one wheel behaved, the other didn't.
- **Root cause:** In motor A's direction logic, the "reverse" branch contained the *same two compare-register assignments* as the forward branch, just written in the opposite order. Reordering lines doesn't reorder signals — both branches drove the motor the same way. Motor B's branches were correctly mirrored, hence the asymmetry.
- **Fix:** Reverse branch rewritten so the channel held at 99 actually swaps between branches (that swap is what reverses the motor).
- **Lesson:** `grep` every `SET_COMPARE` and audit each line against one stated rule. Symmetric-looking code isn't necessarily symmetric.

### 11. Huge dead zone — fast decay vs. slow decay
- **Symptom:** Motors needed a duty of ~60/99 just to start moving, leaving almost no usable control range; small corrections were impossible and the robot "slapped" between overcorrections.
- **Root cause:** The bridge was being driven in fast-decay mode (PWM on one input, other low), where low duty cycles deliver very little average torque against motor friction.
- **Fix:** Switched to slow-decay drive (one input held high, PWM = `99 − duty` on the other). MIN_DUTY dropped from 60 to 10, recovering nearly the full control range. Bonus: at duty 0 slow decay brakes rather than coasts, so the robot stands "planted."
- **Lesson:** Decay mode is a control-system decision, not an implementation detail. It changes the torque-vs-duty curve — and it inverts the meaning of every compare value, so *every* register write (drive branches, zero case, safety stop, leftover test lines) must be converted to the new convention at once. Two follow-up bugs (#10 and a direction-convention flip) came from converting only part of them.

### 12. Sawtooth oscillation — integral windup and a clamp bug
- **Symptom:** Slow, growing lean followed by a sudden lurch, repeating — a sawtooth pattern in the output.
- **Root cause:** The integral term accumulated without proper limits (windup), and the clamp as first written didn't actually bound the stored integral state.
- **Fix:** Proper anti-windup: the integral accumulator itself clamped to a fixed range, not just the final output.
- **Lesson:** Clamping the output while letting the internal state grow unbounded just hides the windup until it's big enough to hurt. Clamp the state.

### 13. Violent vibration while "balanced" — D-term amplifying its own motor noise
- **Symptom:** Robot balanced (angle stable near zero) but vibrated aggressively. Serial log showed `comp_ang` ≈ 0 while the gyro read ±130 °/s alternating every line — the controller was reacting to vibration it was itself creating.
- **Root cause:** Motor vibration fed mechanical noise into the gyro; the D-term amplified it back into the motors — a self-sustaining feedback loop. Classic derivative-kick on a noisy measurement.
- **Fix:** (1) Software low-pass on the gyro: `gyro_f = 0.7*gyro_f + 0.3*gyro_Z_deg`, applied *after* `gyro_Z_deg` is assigned — the first attempt placed the filter line before the assignment, producing an uninitialized-use build warning; (2) Kd reduced to 0.4; (3) the standstill/rest-zone gate switched to use the filtered `gyro_f`.
- **Lesson:** The D-term is a noise amplifier by construction. If the plant vibrates, the derivative input must be filtered — and the log columns (stable angle + oscillating rate) are what distinguish controller-induced vibration from a real fall.

---

## Part 3 — Build & flashing toolchain

### 14. Code changes "not taking effect" — stale .elf flashed
- **Symptom:** Behavior identical after flashing, as if the new code never arrived.
- **Root cause(s):** Editor files not saved before building; and separately, a stuck GDB-server process holding a lock on the output file so the build silently reused the old `.elf`.
- **Fix:** Save-all before build; kill stuck GDB server processes; and the definitive countermeasure — a **version marker** (e.g. `V12`) in the serial print string, bumped on every flash, so a successful flash is *proven* on the serial monitor, never assumed.
- **Lesson:** "Flash succeeded" and "the code I just wrote is running" are different claims. The version marker collapses them into one observable fact.

### 15. Old firmware flashed even when the build failed
- **Symptom:** Build errors scrolled past, yet the flash step ran anyway — programming the previous `.elf`.
- **Root cause:** The build and flash commands were chained with `;` in PowerShell, which runs the second command unconditionally.
- **Fix:** Conditional chaining: `cmake --build build/Debug; if ($?) { <flash command> }` — flash only runs on build success.
- **Lesson:** In any scripted pipeline, later steps must be gated on earlier steps' success. `;` is not `&&`.

### 16. GDB server launching the wrong toolchain — two CubeCLT versions installed
- **Symptom:** Debug sessions failing in confusing ways.
- **Root cause:** STM32CubeCLT 1.21.0 and 1.22.0 were installed simultaneously; the GDB server resolved to the wrong installation.
- **Fix:** Old version uninstalled, paths pointed at 1.22.0.

### 17. Build broken after the CLT uninstall — stale CMake cache
- **Symptom:** After removing the old CubeCLT, the project no longer compiled — the compiler path baked into the CMake cache pointed at the deleted installation.
- **Fix:** Clean reconfigure: `Remove-Item -Recurse -Force build` followed by `cmake --preset Debug`.
- **Lesson:** CMake caches absolute toolchain paths. Any toolchain install/uninstall means a fresh configure.

### 18. ST-Link rejected — outdated probe firmware, then version mismatch
- **Symptom:** First the ST-Link was refused for being outdated; after updating its firmware via STM32CubeProgrammer, the VS Code GDB server refused it with a (misleading) "firmware upgrade required" — the server was now *older* than the probe firmware.
- **Fix:** ST-Link firmware updated via CubeProgrammer; then the VS Code STM32 extension and CubeCLT updated to match. In the interim, flashing went through the CubeProgrammer CLI, which accepted the new firmware.
- **Lesson:** The probe firmware, the GDB server, and the IDE extension are a matched set. Updating one end can break the other; keep them moving together.

### 19. "No device found" on SWD
- **Symptom:** GDB server started and reached the ST-Link, but the STM32 itself didn't answer on the SWD lines.
- **Diagnosis method:** Bare-board test — every wire off (battery, driver, IMU, encoders), USB only, then attempt connection. Connects → the chip is fine and something in the external wiring is loading the SWD/3V3 lines; still dead → check the CN2 jumpers, measure 3V3 rail, try another cable/port.
- **Lesson:** Binary-search the hardware: strip to minimum, confirm the core works, reconnect one subsystem at a time.

### 20. The reliable flash pipeline (final form)
Battery connected, robot on its side, from the project root:

```powershell
cmake --build build/Debug; if ($?) {
  & "C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
    -c port=SWD freq=480 mode=UR reset=HWrst `
    -w build\Debug\robot_blink.elf -v -rst
}
```

Then serial monitor at the configured baud rate, and check the version marker in the first print line.

---

## Meta-lessons from the whole project

1. **The code was never the asset at risk — the hardware was.** Firmware survives every disaster; boards don't. Discipline around live wires is worth more than any amount of clever code.
2. **One convention change touches everything.** Switching decay modes inverted the meaning of every PWM register write and spawned three separate bugs. Convention changes need a full audit, not a spot fix.
3. **Make success observable.** The version marker, the serial print columns, the multimeter checkpoints — every hard bug here fell to *making the invisible visible*, not to staring at code.
4. **Signs and axes are empirical facts,** established by experiment against the physical build, never assumed from a datasheet.
