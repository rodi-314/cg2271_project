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

void processGamepad(ControllerPtr ctl) {
  /*
    Nintendo Switch -> Xbox Button Mapping
    A -> B
    B -> A
    Y -> X
    X -> Y
    */

  // Scale x and y values evenly for UART
  int32_t xShifted = ctl->axisX() / 167;
  int32_t yShifted = -ctl->axisY() / 167;

  // Check if turbo mode is turned on
  bool isTurbo = ctl->b();

  int32_t xMod = abs(xShifted);
  int32_t yMod = abs(yShifted);

  // Send UART command to move robot
  // Stationary by default
  int8_t movePacket = 0x00;
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
    }
  }

  // Play music when 'B' button pressed on Nintendo Switch Controller
  if (ctl->a()) {
    movePacket = 0x80;
  }

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
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();
}
