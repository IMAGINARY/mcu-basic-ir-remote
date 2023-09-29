# Basic Infra-red remote control for Arduino

Build and upload the receiver:
```shell
PLATFORMIO_BUILD_FLAGS="-D BASIC_IR_REMOTE_RECEIVE" pio run -t upload
```

OR

Build and upload the sender:
```shell
PLATFORMIO_BUILD_FLAGS="-D BASIC_IR_REMOTE_SEND" pio run -t upload
```

The sender send a single IR code once after starting up (or after pressing the reset button).
The IR command is configurable in [`src/main.cpp`](./src/main.cpp).

Heavily based on the [`IRRemote` demos](https://github.com/Arduino-IRremote/Arduino-IRremote).

Licensed under the MIT license.
