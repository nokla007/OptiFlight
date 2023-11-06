# OptiFlight for STM32
**Target Device: STM32F446 (cortex-m4)**\
\
To run:\
Go to build directory: `cd build`\
Compile the project: `make all`\
Load into STM32: `make load`

To Debug:\
Build debug binary: `make debug`

## Programming Guideline:
* User Code Directory: `src/userland/` \
* User main function; `src/userland/main.c` \
* External devices (IMU, ESC etc.): `src/userland/device` \
* Modules (kalman filter, pid etc.): `src/userland/modules`  \

