# m2-sim

## Introduction
Welcome to m2-sim, a simulation tool designed for the [RoboCup Soccer Small Size League (SSL)](https://ssl.robocup.org/).

![M2 Simulation Image](docs/images/readme_v5.png)

## Features Summary
- Import custom 3D models
- Get pixel coordinates from the camera mounted on the robot
- Visualize values received via Protobuf by selecting each robot’s data
![collisio_demo](./docs/gif/collision.gif)
Please read [Documents](docs/)
  
## System Requirements
Before diving into the exciting world of RoboCup Soccer SSL with `m2-sim`, ensure your system meets the following requirements:

`CMakeLists.txt` requires exactly three things. Verified with Qt 6.10.3.

- **Qt 6** — Core, Gui, Widgets, Network, Quick, **Quick3D**, **Quick3DPhysics**, Qml.
  Qt Quick 3D and Qt Quick 3D Physics are *Additional Libraries* in the Qt installer
  and are not installed by default. Without them the CMake configure step fails.
- **Boost** — `boost::asio`, used for the UDP senders.
- **Protobuf** — protocol definitions and `protoc`.

Eigen, yaml-cpp, assimp, bullet and vulkan-volk are **not** used anywhere in this
project; earlier versions of this file listed them by mistake.

## Getting Started
To get started with `m2-sim`, follow these steps:

### 1. Install Dependencies

#### Windows (verified)
See **[New_README.md](New_README.md)** for the full Windows procedure. In short:
install Qt 6 with MinGW 64-bit plus Qt Quick 3D and Qt Quick 3D Physics, install
[vcpkg](https://github.com/microsoft/vcpkg) into `C:\ws\vcpkg`, then run
`.\build-windows.ps1`, which installs the vcpkg packages, builds, and deploys the
Qt runtime for you.

#### macOS
```bash
brew update
brew install qt cmake boost protobuf
```
Do not use `boost@1.85` or `protobuf@21`: both formulae have been disabled in
Homebrew (2026-01-08 and 2026-04-05) and can no longer be installed.

#### Ubuntu (not verified)
```bash
sudo apt update
sudo apt install cmake build-essential libboost-all-dev protobuf-compiler libprotobuf-dev
```
Qt must provide Quick 3D and Quick 3D Physics, which the distribution packages may
not include. Installing Qt 6 through the official Qt installer is the safer route.

### 2. 3D Models
The 3D models the simulator needs are committed under `assets/`, so a clone is
ready to run as it is. There is nothing to download.

`src/qml/sim/Field.qml` and `GameObjects.qml` import directories under
`assets/`, and a missing import directory is a fatal QML error, so the
application starts with no window if `assets/` is absent.

To replace the models with your own, see
[how to import custom 3D model](docs/import_model.md). Sample models are also
available here:
[Download 3D Models](https://drive.google.com/drive/folders/17iXSCv_ecgYn4Mx0ziXjV6I9hVO665dg?usp=share_link)


### 3. Building the Project

On **Windows**, run the build script from the repository root. It configures with
Ninja and the vcpkg toolchain, builds, and deploys the Qt runtime:

```powershell
.\build-windows.ps1
```

On **macOS / Linux**:

```bash
cd ssl-RAVEN-sim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### 4. Launch the GUI

```bash
cmake --build build --target run
```

**Start the application from the `build` directory.** It reads
`../src/qml/Main.qml` and `../config/config_v2.ini` as paths relative to the
current directory, so launching it from anywhere else loads nothing and no
window appears. The `run` target above already runs in `build/`; to start the
executable by hand, `cd build` first.

Changing QML does **not** require a rebuild: QML is read from disk at startup, so
edit and restart. Only C++ changes need a rebuild.

### 5. Additional Notes
If the build fails, or it builds but no window appears, see the troubleshooting
table in [New_README.md](New_README.md).

## Documentation

- [New_README.md](New_README.md) — full setup, run and troubleshooting guide
- [docs/architecture.md](docs/architecture.md) — structure, per-tick processing order, protocol reference
- [docs/key_mouse.md](docs/key_mouse.md) — keyboard and mouse controls
- [docs/import_model.md](docs/import_model.md) — replacing the 3D models
- [ai/findings/](ai/findings/) — known defects

## Related Tools
Enhance your RoboCup Soccer SSL experience with these related tools:

- [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller): The official game controller for managing match flow and rules.
- [ssl-autorefs](https://github.com/RoboCup-SSL/ssl-autorefs): Automated referee systems for unbiased and accurate game officiating.

## Contributing
Contributions are welcome. Open an issue or a pull request against
[Rione/ssl-RAVEN-sim](https://github.com/Rione/ssl-RAVEN-sim). Known defects are
tracked in [ai/findings/](ai/findings/), one file per finding.

## License
This project is licensed under the GNU General Public License version 3 (GPL v3). See the [LICENSE](LICENSE) file for details.
