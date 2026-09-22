# PedroSwerving / Phoenix integration

This checkout supplies Pedro Pathing 3.0.1 `core` and `revhub`. Phoenix owns the differential-pod adapter, hardware lifecycle, safety gates, Pinpoint validation, and robot commissioning OpModes in the sibling [robot repository](../Phoenix-Force-10100-Differential-Swerve/README.md).

The robot normally consumes `com.pedropathing:revhub:3.0.1` from Maven Central. No local Maven publishing, copied classes, composite build, or machine-specific source path is required. The upstream generic `Swerve` is not suitable for Phoenix's BRAKE and differential-pod policies; use the robot's `DifferentialSwerveDrivetrain`.

## Build and verify

Use JDK 17 or 21 (verified locally with JDK 21), an Android SDK with platform 35 for this library, and a local `local.properties` containing your `sdk.dir`, or `ANDROID_HOME`. Do not commit SDK paths. The Gradle wrappers retain their existing versions: Pedro 9.0.0 and robot 9.1.0. Both produce Java 8 bytecode; the robot retains FTC SDK 12.0.0 and compile SDK 30.

```powershell
./gradlew.bat :core:test :core:spotlessCheck :revhub:assembleRelease
./scripts/verify-projects.ps1 -BuildJdk 'YOUR_JDK_21_DIRECTORY'
```

The script builds this library, runs the robot tests and debug APK build against the actual local JAR/AAR, then repeats the robot checks using published 3.0.1. It leaves the published-dependency APK and reports as the final outputs. Pass `-RobotRepo 'PATH_TO_ROBOT_REPOSITORY'` when the repositories are not adjacent. `-BuildJdk` is optional when your Gradle/JAVA_HOME configuration already selects a compatible JDK. On Unix, use `./gradlew` and the equivalent commands in the robot's [integration status](../Phoenix-Force-10100-Differential-Swerve/INTEGRATION_STATUS.md).

The local-artifact verification mode explicitly replaces the Maven dependencies for that invocation, preventing duplicate Pedro classes. Rebuild the library before testing changes; stale local binaries are not rebuilt by the robot wrapper.

## Verified contracts and remaining work

The seven `PhoenixContractTest` cases cover field velocity versus body twist, robot-relative manual commands, the allocator's field-to-body conversion, paths, braking-model inversion, and the published follower's stop/completion behavior. The robot adds hardware-boundary fault tests and builds against both dependency modes. See [INTEGRATION_STATUS.md](../Phoenix-Force-10100-Differential-Swerve/INTEGRATION_STATUS.md) for the current evidence and physical commissioning gates.

Pedro 3.0.1 `Follower.stop()` changes state without immediately commanding the hardware; Phoenix's `SafePedroFollower` supplies immediate zero. Foresight advances at the parametric endpoint even if final heading/velocity tolerances are unmet. Phoenix checks terminal pose and velocity independently and labels out-of-tolerance completion as a fault. `timeoutConstraint` is in milliseconds and does not replace the robot's whole-path deadline.

Library control behavior remains compatible with published 3.0.1. The production-source changes clarify the `ManualDrive` coordinate comments and normalize its Spotless formatting; the integration fixes live in the robot adapter. The robot's Pinpoint directions and heading convention are commissioned. Loaded speed/braking characterization remains a separate gate; do not enable Foresight paths until its model values are entered.

[PEDRO_INTEGRATION_PLAN.md](PEDRO_INTEGRATION_PLAN.md) records the original design, rationale, and staged hardware acceptance criteria. Its implementation status header links the completed software to the remaining measurements.
