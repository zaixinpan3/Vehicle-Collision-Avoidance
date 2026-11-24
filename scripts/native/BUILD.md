# Offline AArch64 build

From the repository root, generate C sources for one controller configuration on the development workstation. The optional second argument to `exportArm64Sources` accepts the same configuration overrides as the online controller, including the target operating domain. Use a dedicated output directory.

```sh
matlab -batch "addpath('scripts'); exportArm64Sources('/tmp/pcbf-generated')"
CASADI_ARM64_DIR=/opt/casadi-aarch64 scripts/native/buildArm64Kernels.sh /tmp/pcbf-generated /tmp/pcbf-arm64
```

The cross-build uses `aarch64-linux-gnu-gcc` and `aarch64-linux-gnu-g++` by default. Set `CC` and `CXX` to compiler paths or wrapper scripts to use another sysroot. The target CasADi distribution must have the C++ headers, shared library, IPOPT plugin and its numerical dependencies. The build copies the IPOPT plugin and its target-distribution dependencies into the package. The loader and system libraries come from the target OS. Keep `configuration.mat` with each build to identify its specialized constants.

The track interface in `nrmm_track.h` accepts one timestamped inertial target position per synchronized ego hypothesis. It maintains acquisition history, propagates the observer through dropouts and publishes the motion estimate and error intervals. The prediction library exports Fiala rollout, tangents and the ego sensor-period flow. The geometry library exports target occupancy, the vector-field initialization, rectangle clearance and terminal separation. Generated C interfaces use column-major arrays.

`pcbf_ipopt_trial` runs the CasADi/IPOPT trial search with native value, Jacobian and stop callbacks. The caller owns the frame data and validates any trial before dispatch. The raw-block frame orchestration, complete trajectory oracle, shifted-plan acceptance and actuator dispatch currently live in the MATLAB controller. Completing their native integration is required for an autonomous ROS control node.


To check a build, generate the MATLAB fixtures with `addpath('tests/native'); arm64ParityFixtures('/tmp/pcbf-generated')`. Run `tests/native/checkArm64.sh /tmp/pcbf-arm64 /tmp/pcbf-generated/parity.bin` with the same cross compiler. Set `QEMU_AARCH64` to the emulator and `ARM64_SYSROOT` to the matching ARM64 Linux filesystem for offline execution. The parity test covers observation streams, physical prediction and derivatives, dual geometry, terminal conditions, and the field guide. The solver test exercises the callback ABI on small analytic problems. Board timing and closed-loop scenario acceptance require separate measurements.
