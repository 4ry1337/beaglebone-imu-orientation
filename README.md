# Embedded IMU

Reads an AltIMU-10 v5 on a BeagleBone Black and shows its 3D orientation live in a Qt app.

**Status: finished.** This was a course project at Nazarbayev University.
Pull requests are still welcome.

The BeagleBone Black (BBB) reads the gyroscope, accelerometer and
magnetometer over I2C. It feeds the readings to the Madgwick filter, which
turns them into an orientation quaternion. The BBB then sends the quaternion
over TCP. On the PC, a Qt client draws a box that turns with the sensor. A
terminal client prints the same data.

## Table of Contents

- [Background](#background)
- [Hardware](#hardware)
- [Install](#install)
- [Usage](#usage)
- [Architecture](#architecture)
- [Configuration](#configuration)
- [Project Structure](#project-structure)
- [Known Issues](#known-issues)
- [References](#references)
- [Authors](#authors)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Background

This is Project #3 of CSCI502/702 Hardware/Software Co-Design, Spring 2025.
The handout is in [`docs/course/assignment.pdf`](docs/course/assignment.pdf).
It has six tasks. This repo holds the code for the last three:

| Task | Goal | Code |
| --- | --- | --- |
| 4. IMU interfacing | Switch the sensors on and read raw values over I2C | [`tasks/task4/`](tasks/task4) |
| 5. IMU signal processing | Scale the values and run the Madgwick filter | [`tasks/task5/`](tasks/task5) |
| 6. IMU GUI | Send the orientation to a PC and draw it in Qt | [`server/`](server), [`client/`](client) |

Tasks 1 to 3 cover the BBB setup and need no code here. The group report,
with the register work and screenshots, is in [`docs/report.pdf`](docs/report.pdf).

## Hardware

- BeagleBone Black, connected to the PC with its USB cable
- [Pololu AltIMU-10 v5](https://www.pololu.com/product/2739). It carries:
  - an LSM6DS33 gyroscope and accelerometer at I2C address `0x6B`
  - an LIS3MDL magnetometer at `0x1E`
  - an LPS25H barometer, which this project does not use

Wire the IMU to the BBB's P9 header:

| IMU pin | BBB pin |
| --- | --- |
| SCL | P9_19 |
| SDA | P9_20 |
| GND | P9_1 |
| VDD | P9_3 (3.3 V) |
| VIN | not connected |

P9_19 and P9_20 belong to I2C bus 2, so the code opens `/dev/i2c-2`.

## Install

### Dependencies

On the BBB:

- `g++`, `gcc` and `make`
- `i2c-tools`, if you want to probe the bus by hand

On the PC:

- Qt 6 with the Widgets, OpenGL Widgets and Network modules. Qt Creator is
  the easiest way to build the GUI client.
- `g++` and `make`, for the terminal client

### Steps

1. Clone the repository on the PC:

   ```sh
   git clone https://github.com/4ry1337/embedded-imu.git
   cd embedded-imu
   ```

2. Plug in the BBB. Over USB, it is at `192.168.7.2` and the PC is at
   `192.168.7.1`. The default login is `debian` with the password `temppwd`.

3. Copy the server and the task programs to the BBB:

   ```sh
   scp -r server tasks debian@192.168.7.2:~/
   ```

4. Build the server on the BBB:

   ```sh
   ssh debian@192.168.7.2
   cd server && make
   ```

## Usage

### Task 6: live orientation

Start the server on the BBB first. It waits for a client:

```sh
./build/server
```

Then start a client on the PC.

For the GUI, open [`client/gui/client.pro`](client/gui/client.pro) in Qt Creator
and run it. Click **Connect and Show** to open the 3D view. Click
**Disconnect** to stop. You can also build it from a terminal:

```sh
cd client/gui
qmake6 && make
./client
```

For the terminal client:

```sh
cd client/cli
make
./build/client
```

It prints one line per message, such as `q0=0.98, q1=0.01, q2=-0.12, q3=0.15`.

If the client disconnects, the server waits for the next one.

### Tasks 4 and 5

Build and run these on the BBB:

```sh
cd tasks/task4
gcc read_imu_once.c -o read_imu_once && ./read_imu_once   # one reading
gcc read_imu_loop.c -o read_imu_loop && ./read_imu_loop   # a reading every 0.5 s

cd ../task5
./build.sh && ./quaternion   # prints the quaternion at 50 Hz
```

## Architecture

```text
 AltIMU-10 v5        BeagleBone Black                    PC
┌────────────┐ I2C  ┌──────────────────────────┐ TCP  ┌────────────────────┐
│ LSM6DS33   │ bus 2│ read registers           │54321 │ client/gui (Qt)    │
│ LIS3MDL    ├─────►│ scale to SI units        ├─────►│   or               │
└────────────┘      │ Madgwick filter, 50 Hz   │      │ client/cli         │
                    └──────────────────────────┘      └────────────────────┘
```

The server code lives in [`server/src/`](server/src):

| File | Role |
| --- | --- |
| `I2CDevice` | Opens the bus and reads or writes registers |
| `LSM6DS33`, `LIS3MDL` | Switch a sensor on and read its axes |
| `AltIMU` | Scales the readings and runs the Madgwick filter |
| `SocketServer` | TCP server for one client at a time |
| `server.cpp` | Main loop at a fixed 50 Hz |

The Qt client reads the socket in a `QThread` ([`mythread.cpp`](client/gui/mythread.cpp)).
It hands each quaternion to an OpenGL widget ([`myglwidget.cpp`](client/gui/myglwidget.cpp)),
which redraws every 100 ms.

### Wire format

Each message is one line of plain text, with four numbers and a newline:

```text
q0,q1,q2,q3\n
```

`q0` is the scalar part. The server sends 50 messages a second.

## Configuration

### Sensor settings

The code writes these control words when it starts. The report explains
each bit.

| Sensor | Register | Value | Effect |
| --- | --- | --- | --- |
| LSM6DS33 | `CTRL1_XL` (0x10) | `0x30` | Accelerometer at 52 Hz, ±2 g |
| LSM6DS33 | `CTRL2_G` (0x11) | `0x30` | Gyroscope at 52 Hz, ±245 dps |
| LSM6DS33 | `CTRL9_XL` (0x18) | `0x38` | Accelerometer X, Y and Z on (the default) |
| LIS3MDL | `CTRL_REG1` (0x20) | `0x0C` | 5 Hz, low-power X and Y |
| LIS3MDL | `CTRL_REG2` (0x21) | `0x00` | ±4 gauss |
| LIS3MDL | `CTRL_REG3` (0x22) | `0x00` | Continuous conversion |

The handout asks for 50 Hz and 6.25 Hz. The chips cannot do those rates, so
the code uses the closest ones: 52 Hz and 5 Hz.

### Scale factors

The filter needs physical units. The datasheet sensitivities give:

| Sensor | Sensitivity | Scale factor | Unit |
| --- | --- | --- | --- |
| Accelerometer | 0.061 mg/LSB | 0.061 × 10⁻³ × 9.81 | m/s² |
| Gyroscope | 8.75 mdps/LSB | 8.75 × 10⁻³ × π/180 | rad/s |
| Magnetometer | 6842 LSB/gauss | 1 / 6842 | gauss |

The filter integrates the gyro rate, so it must be in rad/s. It normalizes
the accelerometer and magnetometer vectors, so their units do not matter.

### Fixed values

These values are constants in the code. Change them there:

| Value | Default | Where |
| --- | --- | --- |
| BBB address | `192.168.7.2` | [`client/cli/src/client.cpp`](client/cli/src/client.cpp), [`client/gui/mythread.cpp`](client/gui/mythread.cpp) |
| TCP port | `54321` | the two files above and [`server/src/server.cpp`](server/src/server.cpp) |
| I2C bus | `2` | [`server/src/server.cpp`](server/src/server.cpp) |
| Filter rate | 50 Hz | `SAMPLE_FREQ_HZ` in [`server/src/AltIMU.h`](server/src/AltIMU.h), and `sampleFreq` with `SAMPLE_PERIOD_NS` in `tasks/task5/` |
| Filter gain β | 0.1 | [`server/src/AltIMU.cpp`](server/src/AltIMU.cpp) |

## Project Structure

```text
.
├── client/
│   ├── cli/            # Terminal client
│   └── gui/            # Qt 3D client
├── docs/
│   ├── course/         # Handout and Qt OpenGL guide from the course
│   ├── datasheets/     # LSM6DS33 and LIS3MDL datasheets
│   └── report.pdf      # Group report
├── server/             # BBB server: sensors, filter, TCP
└── tasks/
    ├── task4/          # Raw sensor reads
    └── task5/          # Madgwick filter in the terminal
```

## Known Issues

- The bug fixes made after the course were built and their socket code was
  tested on a PC. They have not run on the BBB with the IMU.
- Older versions never set `CTRL_REG3`, so the magnetometer stayed powered
  down. The filter then ran without it, so the demo in the report did not use
  the compass. Now the magnetometer is on, but it is not calibrated. Nearby
  metal can skew the heading.
- The server serves one client at a time.
- The register read helpers in `tasks/` and `I2CDevice::read_register` return
  `1` on error. A caller cannot tell that apart from a real reading of 1.
- `tasks/task4/read_imu_loop.c` writes the register address from an `int`.
  This works only on little-endian CPUs, such as the BBB's.
- The report links to `task5/quaternions.c`. That file is now
  [`tasks/task5/quaternion.c`](tasks/task5/quaternion.c).
- The demo video linked in the report is not public.

## References

- D. Molloy, *Exploring BeagleBone: Tools and Techniques for Building with
  Embedded Linux*, 2nd ed., Wiley, 2019. Code: <https://github.com/derekmolloy/exploringBB>
- S. Madgwick, open-source IMU and AHRS algorithms:
  <http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/>
- Pololu, [AltIMU-10 v5](https://www.pololu.com/product/2739)
- STMicroelectronics, [LSM6DS33 datasheet](https://www.pololu.com/file/0J1087/LSM6DS33.pdf)
- STMicroelectronics, [LIS3MDL datasheet](https://www.pololu.com/file/0J1089/LIS3MDL.pdf)

## Authors

- Rakhat Yskak
- Amantay Kapyshev
- Ualikhan Tukenov

## Maintainers

Rakhat Yskak: [@4ry1337](https://github.com/4ry1337), yskak.rakhat@gmail.com

## Contributing

Ask questions and report bugs in [GitHub Issues](https://github.com/4ry1337/embedded-imu/issues).
Pull requests are welcome. For a big change, open an issue first so we can
agree on the plan.

## License

[MIT](LICENSE) © 2025 Rakhat Yskak, Amantay Kapyshev, Ualikhan Tukenov

The MIT license does not cover these parts. Their own authors hold the rights:

- [`server/src/SocketServer.cpp`](server/src/SocketServer.cpp) comes from
  Derek Molloy's *Exploring BeagleBone*. Its header puts it under the EUPL 1.1.
  `client/cli/src/SocketClient.*` follows the same book's example, but has no
  license header.
- [`tasks/task5/MadgwickAHRS.c`](tasks/task5/MadgwickAHRS.c) and `.h` are
  Sebastian Madgwick's code. The filter in `server/src/AltIMU.cpp` is a copy
  of it. Its license is unclear: x-io's site is quoted as saying its code is
  GPL unless the source says otherwise, and the source says nothing.
- `docs/datasheets/` belongs to STMicroelectronics.
- `docs/course/` is course material from the instructor.
