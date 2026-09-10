# ZynqDetector

Linux runtime for the Germanium detector on Zynq. It receives commands over ZeroMQ and supports hardware and simulation builds.

## Build

The protocol definition is maintained separately in [Germanium-Communication-Protocol](https://github.com/lijibnl/Germanium-Communication-Protocol). Before building, copy its `GermaniumDetectorProtocol.hpp` and `GermaniumDetectorProtocol.cpp` into `detector/Germanium/`. The Makefile expects both files at that location. The protocol repository is currently under a personal account and is planned to move to the organization.

Install a C++17 compiler, `make`, `pkg-config`, and the libzmq development files. Then build with:

```sh
make SIM_MODE=1  # simulation
# or
make             # hardware
```

Run `make clean` before switching between simulation and hardware builds.

Run the executable with the detector's NELM value:

```sh
./bin/GermaniumDetector 192
# or
./bin/GermaniumDetector 384
```
