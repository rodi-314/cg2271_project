#include <Bluepad32.h>

// Constants
#define RXD2 16
#define TXD2 17
#define CTL_AXIS_BITSHIFT 7
#define LEFT_SPEED_BITSHIFT 3
#define SPEED_THRESHOLD 0
#define LEFT_SPEED_MASK 0x38
#define RIGHT_SPEED_MASK 0x7
#define SPEED_MASK 0x3F
#define DIRECTION_MASK 0x40

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial2.write(0x00);
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

// void dumpGamepad(ControllerPtr ctl) {
//     Serial.printf(
//         "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
//         "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
//         ctl->index(),        // Controller Index
//         ctl->dpad(),         // D-pad
//         ctl->buttons(),      // bitmask of pressed buttons
//         ctl->axisX(),        // (-511 - 512) left X Axis
//         ctl->axisY(),        // (-511 - 512) left Y axis
//         ctl->axisRX(),       // (-511 - 512) right X axis
//         ctl->axisRY(),       // (-511 - 512) right Y axis
//         ctl->brake(),        // (0 - 1023): brake button
//         ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
//         ctl->miscButtons(),  // bitmask of pressed "misc" buttons
//         ctl->gyroX(),        // Gyro X
//         ctl->gyroY(),        // Gyro Y
//         ctl->gyroZ(),        // Gyro Z
//         ctl->accelX(),       // Accelerometer X
//         ctl->accelY(),       // Accelerometer Y
//         ctl->accelZ()        // Accelerometer Z
//     );
// }

// int32_t limit(int32_t integer) {
//     if (integer > 0) {
//         return integer - 1;
//     } else if (integer < 0) {
//         return integer + 1;
//     }
//     return integer;
// }

// int32_t positive(int32_t integer) {
//     if (integer < 0) {
//         return 0;
//     }
//     return integer;
// }

void processGamepad(ControllerPtr ctl) {
  /*
    Nintendo Switch -> Xbox Button Mapping
    A -> B
    B -> A
    Y -> X
    X -> Y
    */

  // Move robot when joystick is moved
  // Serial.printf("x original: %4d\n", ctl->axisX());
  // Serial.printf("y original: %4d\n", ctl->axisY());

  // // Scale x specifically
  // int32_t x = ctl->axisX();
  // int32_t xShifted = 0;
  // if (x > 500) {
  //   xShifted = 3;
  // } else if (x > 400) {
  //   xShifted = 2;
  // } else if (x > 125) {
  //   xShifted = 1;
  // } else if (x > -125) {
  //   xShifted = 0;
  // } else if (x > -400) {
  //   xShifted = -1;
  // } else if (x > -500) {
  //   xShifted = -2;
  // } else {
  //   xShifted = -3;
  // }

  // // Scale y evenly
  // int32_t yShifted = -ctl->axisY() / 167;

  // Scale x and y values evenly for UART
  int32_t xShifted = ctl->axisX() / 167;
  int32_t yShifted = -ctl->axisY() / 167;

  // // Bitshift and limit x and y values for UART
  // int32_t xShifted = limit(ctl->axisX() >> CTL_AXIS_BITSHIFT);
  // int32_t yShifted = -limit(ctl->axisY() >> CTL_AXIS_BITSHIFT);

  // Check if turbo mode is turned on
  bool isTurbo = ctl->b();

  int32_t xMod = abs(xShifted);
  int32_t yMod = abs(yShifted);
  // Serial.printf("x: %4d\n", xShifted);
  // Serial.printf("y: %4d\n", yShifted);

  // if (xMod > SPEED_THRESHOLD || yMod > SPEED_THRESHOLD) {
  //     int32_t max = xMod;
  //     if (yMod > xMod) {
  //       max = yMod;
  //     }
  //     ctl->playDualRumble(0, 250, (1 << (max + 4 + isTurbo)) - 1, (1 << (max + 4 + isTurbo)) - 1);
  // }

  // Send UART command to move robot
  // Stationary by default
  int8_t movePacket = 0x00;
  /*
    // Move forward by default if x is above the speed_threshold
    if (xMod > SPEED_THRESHOLD) {
      movePacket = (movePacket & ~SPEED_MASK) + (xMod << LEFT_SPEED_BITSHIFT) + xMod;
    }

    // Move forward at specified speed
    if (yShifted < -SPEED_THRESHOLD) {
      movePacket = (movePacket & ~SPEED_MASK) + (yMod << LEFT_SPEED_BITSHIFT) + yMod;
    }

    // Move backward at specified speed
    if (yShifted > SPEED_THRESHOLD) {
      movePacket = ((movePacket | DIRECTION_MASK) &~ SPEED_MASK) + (yMod << LEFT_SPEED_BITSHIFT) + yMod;
    }
    Serial.println(movePacket);

    // Move left
    if (xShifted < -SPEED_THRESHOLD) {
      int32_t leftSpeed = (movePacket & LEFT_SPEED_MASK) >> LEFT_SPEED_BITSHIFT;
      movePacket = (movePacket & ~LEFT_SPEED_MASK) + (positive(leftSpeed - (xMod >> 1)) << LEFT_SPEED_BITSHIFT);
      Serial.printf("Left: %d\n",leftSpeed);

    // Move right
    } else if (xShifted > SPEED_THRESHOLD) {
      int32_t rightSpeed = (movePacket & RIGHT_SPEED_MASK);
      movePacket = (movePacket & ~RIGHT_SPEED_MASK) + positive(rightSpeed - (xMod >> 1));
      Serial.printf("Right: %d\n",rightSpeed);
    }
*/
  int32_t leftSpeed = yShifted + xShifted;
  int32_t rightSpeed = yShifted - xShifted;

  if (yShifted < 0) {
    leftSpeed = yShifted - xShifted;
    rightSpeed = yShifted + xShifted;
  }

  if (leftSpeed < 0) {
    leftSpeed = max(-3, leftSpeed);
  } else {
    leftSpeed = min(3, leftSpeed);
  }
  if (rightSpeed < 0) {
    rightSpeed = max(-3, rightSpeed);
  } else {
    rightSpeed = min(3, rightSpeed);
  }

  // Set turbo mode if 'A' button is pressed on the Nintendo Switch Controller
  movePacket = (((rightSpeed < 0) << 2) | (abs(rightSpeed) & 0x3)) | ((((leftSpeed < 0) << 2) | (abs(leftSpeed) & 0x3)) << 3) | (ctl->b() << 6);
  // Serial.printf("Left: %d\n",leftSpeed);
  // Serial.printf("Right: %d\n",rightSpeed);

  // Joystick is not pressed
  if (!xMod && !yMod) {
    // Move robot when dpad is pressed
    switch (ctl->dpad()) {

      // Dpad is not pressed
      case 0x00:
        movePacket = 0x00;
        break;

      // Up button pressed
      case 0x01:
        movePacket = 0x1B;
        break;

      // Down
      case 0x02:
        movePacket = 0x3F;
        break;

      // Right
      case 0x04:
        movePacket = 0x1F;
        break;

      // Left
      case 0x08:
        movePacket = 0x3B;
        break;
    }

    if (movePacket > 0 && isTurbo) {
      movePacket |= (1 << 6);
      // ctl->playDualRumble(0, 250, 255, 255);
    } else if (movePacket > 0) {
      // ctl->playDualRumble(0, 250, 127, 127);
    }
  }

  // Play music when 'B' button pressed on Nintendo Switch Controller
  if (ctl->a()) {
    movePacket = 0x80;
  }

  // dumpGamepad(ctl);
  // Serial.println(movePacket);
  Serial2.write(movePacket);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.write(0x00);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  // BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  //delay(150);
}
