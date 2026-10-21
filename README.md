# Go2 hardware interface

Hardware Abstraction Layer for Go2 robot

- `go2_hal/`: thin wrapper around the Unitree SDK2 (`LowLevelInterface`): publishes `rt/lowcmd`,
  subscribes to `rt/lowstate`, and switches off the Unitree motion service.
- `src/go2_robot_hw.cpp`: the `ros_control` `RobotHW` (`Go2RobotHw`) built on top of it.

## Handing the motors over from the Unitree SDK

When the Go2 boots, its own motion service (the "sport" mode) drives the motors. Our controllers
take over by sending raw torque/PD commands on `rt/lowcmd`. The Go2 is picky about this handover,
and three separate problems made the motors go limp (or never respond) at start-up. Each one is
handled in `go2_hal`. This section explains why the code looks the way it does, so nobody
"simplifies" it back into the bug.

### 1. The motion service must really be released

While the motion service runs, the robot **silently discards every `rt/lowcmd`**.
`LowLevelInterface::Init()` calls `MotionSwitcherClient::ReleaseMode()` and checks the result with
`CheckMode()`.

- `CheckMode()` can fail (timeout, DDS not ready yet). When it fails, the motion name it returns is
  empty, which looks exactly like "no service running". `queryServiceStatus()` therefore returns
  `-1` on failure, separate from `0` (released) and `1` (still running). A failed query used to be
  read as "released", so we went on to send commands the robot was ignoring.
- The release loop is bounded (`max_release_attempts`). At the end it prints a clear verdict:
  released, still running, or unconfirmed. If it is still running, release the service from the
  remote or power-cycle the robot.

### 2. The command stream must never stop (motor watchdog)

Once the motion service is released, the Go2 **cuts motor power if `rt/lowcmd` goes quiet for more
than about 20 ms**. Only the motion service can re-energise them, and we have just switched it off.
If the stream stops once, the motors stay idle until the robot is restarted.

Start-up used to have two such gaps:

- the release loop used `sleep(5)` between attempts;
- after `init()`, nothing publishes until the controller manager calls the first `write()`.
  Creating the publishers and loading the controllers takes much longer than 20 ms.

Fixes:

- `keepMotorsAlive(seconds)` sends zero-torque commands (`kp = kd = tau = 0`) at 500 Hz, blocking.
  It replaces the sleeps in the release loop and runs for another 0.5 s after the release is
  confirmed.
- `startKeepAlive()` / `stopKeepAlive()` run the same zero-torque stream in a background thread.
  `Go2RobotHw::init()` starts it right after the start-up routine. The first `Go2RobotHw::write()`
  stops it and takes over the stream. `stopKeepAlive()` is idempotent and costs only an atomic
  check after the first call, so calling it every cycle is fine. The destructor also stops it.

Zero-torque commands are safe: the joints are passive, exactly as they would be in a damping-free
idle. Do not add logic that pauses publishing (e.g. waiting for a controller, or skipping `write()`
while something initialises) unless the keep-alive covers that gap.

### 3. The CRC must be computed over zeroed padding

Every `LowCmd_` carries a CRC32. The robot drops frames whose CRC does not match, **without any
error message**. The Unitree examples compute it as
`crc32_core((uint32_t*)&cmd, sizeof(LowCmd_)/4 - 1)`. This covers the raw struct memory,
**padding bytes included**. DDS does not send the padding, so the robot recomputes the CRC with the
padding set to zero.

The Unitree examples work because their `LowCmd_` happens to be zero-initialised. In
`Go2RobotHw`, `go2_lowcmd_` is a member of a heap-allocated object, so its padding can hold leftover
heap bytes. The CRC then never matches, every frame is dropped, and the motors fall back to idle.
Whether this happens depends on the process's allocation history: the same code works in one run
or build and fails in the next, which made it very hard to track down.

`SendLowCmd()` now computes the CRC with `lowCmdCrc()`. It copies the command field by field into
a `memset`-zeroed `LowCmd_` and takes the CRC of that copy. **If the `LowCmd_` IDL changes (fields
added or removed), update `lowCmdCrc()` to match.** A missing field gives wrong CRCs for any
command that sets it.

### Symptoms → likely cause

| Symptom | Likely cause |
|---|---|
| Robot keeps standing or walking under sport mode, commands have no effect | Motion service not released (1) |
| Motors are stiff briefly, then go limp right after start-up and never come back | Gap in the `rt/lowcmd` stream (2) |
| `rt/lowstate` is fine but motors never respond, varies between runs or builds | Bad CRC from dirty padding (3) |
