# Biobuzz 2027 — GNCE Mercury

FTC robot controller code for the DECODE season. Built on the FIRST Tech Challenge SDK 11, with
[Pedro Pathing](https://pedropathing.com/) for path following, [SolversLib](https://docs.seattlesolvers.com/)
for the command scheduler, and [Panels](https://panels.bylazar.com/) for live tuning and telemetry.

```
FtcRobotController/   the stock SDK module - don't edit
TeamCode/             everything we write
  src/main/java/.../config/     subsystems, commands, shooting + localization maths
  src/main/java/.../opmodes/    autos, teleops, and the on-robot device checks
  src/test/java/                local JVM tests (run on your laptop)
```

See [CLAUDE.md](CLAUDE.md) for how the code fits together — the OpMode/`MyRobot` lifecycle, the
shooting pipeline, alliance mirroring, and the known defects.

---

## 1. Set up your machine

### Android Studio

Install **Android Studio Ladybug (2024.2.1) or newer**. The project builds with Android Gradle
Plugin 8.7.0, which older versions of Studio refuse to open.

Use the JDK that ships with Studio (JetBrains Runtime 21). Nothing else needs installing — the
Gradle wrapper in this repo fetches Gradle 8.9 itself on the first build.

### Android SDK

In Studio: **Settings → Languages & Frameworks → Android SDK**, and install

- **SDK Platform 35** (the project compiles against API 35)
- **Android SDK Build-Tools 35.x**
- **Android SDK Platform-Tools** (this is where `adb` comes from)

No NDK is needed. The project has no native code of its own, and the prebuilt `.so` files inside the
FTC libraries are packaged as they are.

### `local.properties`

This file points Gradle at your Android SDK. It is **not** in git — it is specific to your machine —
so create it in the repo root if Studio has not already:

```properties
sdk.dir=/Users/<you>/Library/Android/sdk
```

On Windows: `sdk.dir=C:\\Users\\<you>\\AppData\\Local\\Android\\Sdk` (double backslashes).

Opening the project in Android Studio normally writes this for you.

### Network access on the first build

Dependencies come from the repositories declared in `build.dependencies.gradle`: Maven Central and
Google for the FTC SDK and AndroidX, `repo.dairy.foundation` for SolversLib, and
`mymaven.bylazar.com` for Panels. The last two are not mirrored anywhere, so they have to be
reachable. The first build downloads a few hundred MB — do it on a decent connection well before
competition day; after that Gradle works from its cache.

---

## 2. Build

From the repo root (or use the equivalent buttons in Android Studio):

```bash
./gradlew :TeamCode:assembleDebug      # build the robot controller APK
./gradlew clean                        # when a build gets into a weird state
```

The APK lands in `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk` (around 75 MB — most of it is
OpenCV and the AprilTag native libraries).

After `clean`, a full rebuild takes well under a minute once the dependency cache and the Gradle
daemon are warm. The first build of the day is slower.

### Deploy to the robot

1. Power on the Control Hub and connect your laptop to its Wi-Fi network.
2. `adb connect 192.168.43.1:5555` — or plug in over USB, which needs no `adb connect`.
3. Press **Run** in Android Studio (or `adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`).

OpModes register themselves through the `@TeleOp` and `@Autonomous` annotations, so a new OpMode
appears on the Driver Station as soon as the new APK is installed — nothing to add to a list.

---

## 3. Tests

Three kinds, answering three different questions.

### Local JVM tests — "is the maths right?"

These run on your laptop in a couple of seconds. No robot, no Control Hub. They cover the pure
logic: the shot planner's lookup table and projectile solve, the pose filter, the PIDF controller,
the SAT collision test, the sensor debouncers and the cached motor/servo wrappers.

```bash
./gradlew :TeamCode:testDebugUnitTest

# one class
./gradlew :TeamCode:testDebugUnitTest --tests "*ShotPlannerTest"

# one test method
./gradlew :TeamCode:testDebugUnitTest --tests "*PoseEKFTest.headingCorrectionTakesTheShortWayRound"
```

Two ready-made run configurations are checked in under `.run/`, so they show up in the Run dropdown
after a Gradle sync: **All unit tests** and **ArtifactDataSmootherTest** (the second one is the
template to copy — duplicate it and change the `--tests` filter). These are Gradle run
configurations, so they work the same in Android Studio and in IntelliJ.

Once the project has synced you can also click the green arrow next to any class in
`TeamCode/src/test/java`. If the gutter arrow is missing, the Gradle sync has not finished or has
not run — **File → Sync Project with Gradle Files**. The IDE needs the sync to learn that
`src/test/java` is a test source root; nothing in the Gradle files has to change.

The tests print telemetry-style tables as they go, so the output is readable rather than just a pass
count — the shot planner prints its whole RPM table, the PIDF test prints the loop settling. Reports
also land in `TeamCode/build/reports/tests/testDebugUnitTest/index.html`.

> **Two tests fail on purpose.** `PoseEKFTest.visionCorrectionSurvivesTheNextOdometryUpdate` and
> `ShotPlannerTest.everyDistanceInTheTableHasAReachableHoodAngle` are red because they have found
> real bugs, not because they are broken. Each one's failure message and comments explain what is
> wrong; the details are in [CLAUDE.md](CLAUDE.md) under *Tests*. Expect **64 tests, 2 failed** —
> anything else means something changed.

### Command walkthroughs — "how does the robot actually work?"

**Start here if you are new to this codebase.** One OpMode per subsystem under the **Command Test**
group, in `opmodes/auto/commands/`. Each one is a short, readable command sequence that drives a
single system and logs a timestamped trace on the Driver Station, so you can read the code and then
watch it happen.

Read them in this order — they build up. On the Driver Station they are listed as `C1 …` through
`C9 …` so the list sorts the same way.

| Class | Teaches |
|---|---|
| `IntakeCommandTest` | Set a mode, wait, set it back — the shape of every sequence |
| `TransferCommandTest` | Reusing a command class, `withTimeout`, `addRequirements` |
| `ShooterCommandTest` | `WaitUntilCommand` on a real condition; why the shooter needs the planner |
| `TurretCommandTest` | Targets instead of angles; what feeds a subsystem each loop |
| `DoorCommandTest` | Commands that include their own settling time; default commands |
| `LiftHoldCommandTest` | The PTO clutch: the lift *is* the drivetrain in another gear |
| `ShootSequenceCommandTest` | The whole scoring chain: pose → distance → RPM + hood → ready → fire |
| `FollowPathCommandTest` | How path following works, and when the pose is seeded |
| `LimelightCommandTest` | The camera's two pipelines and who consumes each |

`CommandTestOpMode` carries the explanation of the OpMode lifecycle and the command framework —
worth reading before the tests themselves. Each test's class comment explains what to look for and
the gotchas specific to that subsystem.

These run real mechanisms. The shooter tests spin the flywheel and the path test drives the robot
12 inches, so read the note on the init screen before pressing PLAY. The log also goes to
`adb logcat -s CommandTest`.

### On-robot device checks — "is the hardware wired up?"

One OpMode per subsystem, listed under the **Device Check** group on the Driver Station and numbered
in the order to run them. Select one, press PLAY, read the pass/fail list.

| OpMode | Checks | Moves? |
|---|---|---|
| `00 Check: all devices` | Every expected device name is in the config, plus battery voltage | no |
| `01 Check: drivetrain` | Each motor spins, direction, encoder ticks, current draw | wheels |
| `02 Check: pinpoint odometry` | I2C, status, update rate, both pods counting | no, you push the robot |
| `03 Check: turret` | Encoder reads and tracks; both servos drive the output | a few degrees |
| `04 Check: intake and transfer` | Both motors each way, current draw, pitch servo | rollers |
| `05 Check: shooter` | Both flywheel motors pull, both encoders read, hood sweeps | flywheel at 25% |
| `06 Check: storage beam breaks` | Each receiver follows its emitter | no |
| `07 Check: shooter door` | Cycles closed - open - closed | door |
| `08 Check: limelight` | Streaming, connected, every pipeline returns frames | no |
| `09 Check: lift clutch (PTO)` | Clutch shifts both ways, left disengaged | clutch |

Start with **00** — if the robot configuration is wrong there is no point running the rest.

Anything that moves waits for an **A** press first (**B** skips it), so selecting the wrong OpMode
cannot surprise anyone. **Put the robot on blocks before running `01 Check: drivetrain`** — it spins
the wheels one at a time.

Results also go to the robot log under the tag `DeviceCheck`, so a failure can be read back
afterwards with `adb logcat -s DeviceCheck`.

---

## 4. Live tuning

Most tuning constants are `public static` fields annotated `@Configurable`, which means Panels can
edit them while an OpMode runs — flywheel PIDF, hood limits, turret offsets, shot latencies, the
storage debounce windows. Connect to the robot's Wi-Fi and open Panels in a browser to change them
without rebuilding.

Values edited in Panels are **not** saved back to the source. Once a number is right, put it in the
code.

---

## 5. Troubleshooting

**"Plugin requires Android Studio ... or newer"** — update Android Studio to Ladybug or later.

**`SDK location not found`** — missing or wrong `local.properties`, see above.

**Dependency resolution fails** — check you are online and that `repo.dairy.foundation` and
`mymaven.bylazar.com` are reachable; they are not mirrored on Maven Central.

**A device check reports a device as missing** — the Driver Station robot configuration does not
match what the code asks for. Every expected name is listed in
`TeamCode/src/main/java/.../opmodes/auto/base/Devices.java`.

**No green arrow next to a test class, or the test classes look like plain files** — the project has
not been imported as a Gradle project. Run **File → Sync Project with Gradle Files**. If the IDE
still will not find the tests, the `.run/` configurations work regardless, and so does
`./gradlew :TeamCode:testDebugUnitTest` from a terminal.

**"Project already linked" / duplicate modules on sync** — `.idea/gradle.xml` is linking a module
directory as its own Gradle project. There should be exactly one `<GradleProjectSettings>` block,
with `externalProjectPath` pointing at `$PROJECT_DIR$`. This happens if the project is opened before
the root Gradle files exist, which is how the repo arrived. Deleting `.idea/` and reopening the repo
root also fixes it — nothing in `.idea/` is tracked in git.

**Gradle daemon runs out of memory** — `gradle.properties` ships the FTC default of
`-Xmx1024M`. Raise it to `-Xmx2048M` if the daemon dies mid-build.
