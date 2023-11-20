/* Authorship of this code:
Copyright © 2023 Julian Taffa
This was written as part of the Engineering Capstone:
'Exploring Control Algorithms for Electric Motors in Exoskeleton Gloves'
Within FEIT at the University of Technology Sydney
Under the supervision of Dr Michael Behrens

This work is entirely free, and you can redstribute it and/or modify it 
under the terms of the Creative Commons Zero v1.0 Universal (CC0 1.0) Public Domain Dedication,
which means this code comes with NO warranty and without copyright restrictions.
This notice and the CC0 1.0 Universal dedication should be included in all copies or substantial portions of the software.
 */

/* FlexCAN_T4 Library attribution:

MIT License

Copyright (c) 2019 Antonio Brewer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
*/

#include <FlexCAN_T4.h>  // using FlexCAN library
#include <cstring>       // for access to memcpy

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;  // These Maxon components use the CAN 2.0 standard, not FD

// For the Network Management Protocol (NMT)
#define NMT_Start 0x01
#define NMT_Stop 0x02
#define NMT_PRE_OPERATIONAL 0x80
#define NMT_RESET_NODE 0x81

// Node-ID of 6 for Teensy
#define TEENSY_NODE_ID 0x06

#define SDO_CLIENT_TO_SERVER_BASE 0x0600  // + Motor Node-ID within each function
#define SDO_SERVER_TO_CLIENT_BASE 0x0580  // + Motor Node-ID within each function

#define SDO_READ_COMMAND 0x40  // Command Specifier for SDO read

// For the Controlword
#define SDO_WRITE_CONTROLWORD_INDEX 0x6040
#define SDO_WRITE_CONTROLWORD_SUBINDEX 0x00

// Modes of Operation
#define MODES_OF_OPERATION_INDEX 0x6060
#define MODES_OF_OPERATION_SUBINDEX 0x00
#define PROFILE_POSITION_MODE 1
#define CYCLIC_SYNCHRONOUS_POSITION_MODE 8

// For the Statusword
#define SDO_READ_STATUSWORD_INDEX 0x6041
#define SDO_READ_STATUSWORD_SUBINDEX 0x00

// To Read Current Position
#define SDO_READ_POSITION_INDEX 0x6064
#define SDO_READ_POSITION_SUBINDEX 0x00

// To Write Target Position
#define SDO_WRITE_POSITION_COMMAND 0x22
#define SDO_WRITE_TARGET_POSITION_INDEX 0x607A
#define SDO_WRITE_TARGET_POSITION_SUBINDEX 0x00

#define SDO_WRITE_CONFIRMATION_COMMAND 0x60

// To set Home Position
#define HOME_POSITION_INDEX 0x30B0
#define HOME_POSITION_SUBINDEX 0x00

// To Read Error Codes
#define ERROR_CODE_INDEX 0x603F
#define ERROR_CODE_SUBINDEX 0x00

// For SYNC
#define SYNC_COB_ID 0x80

// Maximal Following Error
#define MAXIMAL_FOLLOWING_ERROR_INDEX 0x6065
#define MAXIMAL_FOLLOWING_ERROR_SUBINDEX 0x00

/// USER-ADJUSTABLE PARAMETERS ///

const int32_t errorValue = 200000;     // Return value if unable to read a proper position
const unsigned long timeout = 100;     // Timeout in milliseconds waiting for CAN message
const unsigned long syncPeriod = 100;  // SYNC period in milliseconds
const int32_t homingIncrement = 2000;  // Increment of steps for homing sequence
const int32_t motorTravel = 135000;    // Number of increments from extreme to extreme


int32_t motorZeroPoint[6];     // Array has 6 elements, but element 0 will be ignored. Elements 1 to 5 will be each motor
int32_t motorMaximumPoint[6];  // Array has 6 elements, but element 0 will be ignored. Elements 1 to 5 will be each motor

// This enum makes the particular finger being referred to easier to read in code
enum Fingers {
  THUMB = 1,  // Node-ID 1
  INDEX,      // Node-ID 2
  MIDDLE,     // Node-ID 3
  RING,       // Node-ID 4
  PINKIE      // Node-ID 5
};

// This enum collects all of the relevant Controlwords together
enum ControlwordCommand {
  SHUTDOWN = 0x06,                        // 0b00000110
  SWITCH_ON = 0x07,                       // 0b00000111
  SWITCH_ON_AND_ENABLE_OPERATION = 0x0F,  // 0b00001111
  DISABLE_VOLTAGE = 0x00,                 // 0b00000000
  QUICK_STOP = 0x02,                      // 0b00000010
  ENABLE_OPERATION = 0x0F,                // 0b00001111
  FAULT_RESET = 0x80                      // 0b10000000
};

enum ProfilePositionControlwords {
  ABSOLUTE_POSITION = 0x001F,
  ABSOLUTE_POSITION_IMMEDIATE = 0x003F,
  RELATIVE_POSITION = 0x007F,
  RELATIVE_POSTION_IMMEDIATE = 0x005F,
  NEW_POSITION = 0x000F,  // Toggle 'New Position'
};

// This enum collects all of the relevant Statuswords together
enum Statusword {
  NOT_READY_TO_SWITCH_ON_MASK = 0x004F,  // 0b000001001111
  SWITCH_ON_DISABLED_MASK = 0x0040,      // 0b000001000000
  READY_TO_SWITCH_ON_MASK = 0x0021,      // 0b000000100001
  SWITCHED_ON_MASK = 0x0023,             // 0b000000100011
  OPERATION_ENABLED_MASK = 0x0027,       // 0b000000100111
  QUICK_STOP_ACTIVE_MASK = 0x0007,       // 0b000000000111
  FAULT_REACTION_ACTIVE_MASK = 0x000F,   // 0b000000001111
  FAULT_MASK = 0x0008                    // 0b000000001000
};

// This function sends the Network Management Procols (NMT)
void sendNMT(uint8_t nodeID, uint8_t command) {
  CAN_message_t msg;
  msg.id = 0x000;  // NMT Command uses ID 0
  msg.len = 2;
  msg.buf[0] = command;
  msg.buf[1] = nodeID;
  Can1.write(msg);
}

void resetMotor(uint8_t nodeID, uint8_t modeOfOperation) {
  writeControlwordSDO(nodeID, SHUTDOWN);                             // Transitions to "Ready To Switch On" state
  delay(20);                                                         // Delay to ensure command is processed
  setModeOfOperation(nodeID, modeOfOperation);                       // set mode
  delay(20);                                                         // Delay to ensure command is procesed
  if (modeOfOperation == PROFILE_POSITION_MODE) {                    // If Profile Position Mode is selected
    writeControlwordSDO(nodeID, SWITCH_ON);                          // Transitions to "Switched On" state
    delay(20);                                                       // Delay to ensure command is processed
    writeControlwordSDO(nodeID, ENABLE_OPERATION);                   // Transitions to "Operation Enabled" state
    delay(20);                                                       // Delay to ensure command is processed
  } else if (modeOfOperation == CYCLIC_SYNCHRONOUS_POSITION_MODE) {  // If Cyclic Synchronous Position Mode is selected
    writeControlwordSDO(nodeID, 0x0B);                               // write the correct Controlword
    delay(20);                                                       // Delay to ensure command is processed
  }
}

// This function reads the Statusword from a given motor driver
uint16_t readStatusword(uint8_t nodeID) {
  uint8_t sdoReadCommand[8] = {
    SDO_READ_COMMAND,
    (uint8_t)(SDO_READ_STATUSWORD_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(SDO_READ_STATUSWORD_INDEX >> 8),    // Index HighByte
    SDO_READ_STATUSWORD_SUBINDEX,                 // Subindex
    0x00, 0x00, 0x00, 0x00                        // The rest are zeroes
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;  // SDO client-to-server COB-ID for specified Node-ID
  outMsg.len = 8;                                  // Data length code for CANopen is 8 bytes
  memcpy(outMsg.buf, sdoReadCommand, 8);           // Copy the SDO command into the message buffer

  // Send the SDO read command
  if (Can1.write(outMsg)) {
    //Serial.print("Sent Statusword read request to motor driver with Node-ID: ");
    //Serial.println(nodeID);
  } else {
    Serial.print("Failed to send Statusword read request to motor driver with Node-ID: ");
    Serial.println(nodeID);
    return 0xFFFF;  // Indicate send failure
  }

  // Now waiting for the response
  CAN_message_t inMsg;
  unsigned long timeout = millis() + 1000;  // set 1sec timeout

  while (millis() < timeout) {
    if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
      // Check if the received message is the SDO response for the Statusword
      if (inMsg.buf[1] == (uint8_t)(SDO_READ_STATUSWORD_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(SDO_READ_STATUSWORD_INDEX >> 8) && inMsg.buf[3] == SDO_READ_STATUSWORD_SUBINDEX) {
        // Construct the Statusword from the response bytes (it will be 16-bit)
        uint16_t statusword = inMsg.buf[4] | (inMsg.buf[5] << 8);
        Serial.print("Node ");
        Serial.print(nodeID);
        Serial.print(" Statusword: ");
        Serial.println(statusword, BIN);
        return statusword;  // Return the Statusword value
      }
    }
  }
  // If this point is ever reached, the response was not received in time
  Serial.print("Response from Node-ID ");
  Serial.print(nodeID);
  Serial.println(" was not received in time.");
  return 0xFFFF;  // The maximum 16-bit number will be the error code
}

// This function writes to the Controlword, to enable the motor, switch on and enable operation
void writeControlwordSDO(uint8_t nodeID, uint16_t controlword) {
  uint8_t sdoWriteCommand[8] = {
    0x2B,                                           // SDO command specifier for writing 2 bytes
    (uint8_t)(SDO_WRITE_CONTROLWORD_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(SDO_WRITE_CONTROLWORD_INDEX >> 8),    // Index HighByte
    SDO_WRITE_CONTROLWORD_SUBINDEX,                 // Subindex
    (uint8_t)(controlword & 0xFF),                  // Controlword LowByte
    (uint8_t)(controlword >> 8),                    // Controlword HighByte
    0x00, 0x00                                      // Rest are reserved
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;
  outMsg.len = 8;
  memcpy(outMsg.buf, sdoWriteCommand, 8);

  if (Can1.write(outMsg)) {
    //Serial.print("Controlword written to motor driver with Node-ID: ");
    //Serial.println(nodeID);

    // Now waiting for the confirmation
    CAN_message_t inMsg;
    unsigned long timeout = millis() + 1000;  // Set a 1-second timeout for the response

    while (millis() < timeout) {
      if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
        // Check if the received message is the SDO write confirmation
        if (inMsg.buf[0] == SDO_WRITE_CONFIRMATION_COMMAND && inMsg.buf[1] == (uint8_t)(SDO_WRITE_CONTROLWORD_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(SDO_WRITE_CONTROLWORD_INDEX >> 8) && inMsg.buf[3] == SDO_WRITE_CONTROLWORD_SUBINDEX) {
          //Serial.println("Controlword write confirmation received.");
          return true;  // Write confirmation received, operation successful
        }
      }
    }
    Serial.println("Controlword write confirmation not received.");
    return false;  // Write confirmation not received, operation failed
  } else {
    Serial.print("Failed to write Controlword motor driver with Node-ID: ");
    Serial.println(nodeID);
    return false;  // Sending write request failed
  }
}

// Function to send an SDO to set the mode of operation, will be used to enable profile position or cyclic synchronous postion mode
bool setModeOfOperation(uint8_t nodeID, uint8_t mode) {
  uint8_t sdoWriteCommand[8] = {
    0x2F,                                        // SDO command specifier for writing 1 byte
    (uint8_t)(MODES_OF_OPERATION_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(MODES_OF_OPERATION_INDEX >> 8),    // Index HighByte
    MODES_OF_OPERATION_SUBINDEX,                 // Subindex
    mode,                                        // Modes of Operation value to write
    0x00, 0x00, 0x00                             // Unused bytes for one-byte write
  };

  // Prepare the CAN message
  CAN_message_t msg;
  msg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;  // COB-ID for SDO Rx is 0x600 + node ID
  msg.len = 8;                                  // Standard SDO frame length
  memcpy(msg.buf, sdoWriteCommand, 8);

  // Send the SDO message
  return Can1.write(msg);
}

// Function to determine if motor is ready or not
bool isMotorReady(uint16_t statusword) {
  // If the fault bit is set, the motor has a fault and is not ready
  if ((statusword & FAULT_MASK) != 0) {
    Serial.println("Motor has a fault!");
    return false;
  }

  // If the 'Operation enabled' bits are set, the motor is ready
  if ((statusword & OPERATION_ENABLED_MASK) == OPERATION_ENABLED_MASK) {
    Serial.println("Motor is ready and enabled for operation.");
    return true;
  }

  Serial.println("Motor is not ready.");
  return false;
}

void setup() {
  Serial.begin(115200);  // high baud rate ensures good balance between speed and reliability
  // while (!Serial); // This line is optional, it would be enabled if the system was to always be connected via USB to a computer.

  // Initialise the Can1 at 250 kbps
  Can1.begin();
  delay(100);  // short delay to allow system to initialise
  Can1.setBaudRate(250000);


  Serial.println("");
  Serial.println("Can1 Initialised Successfully.");

  // Reset and start all motor nodes
  for (int i = THUMB; i <= PINKIE; ++i) {  // For each of the 5 fingers
    sendNMT(i, NMT_RESET_NODE);            // Reset each node
    delay(1000);                           // Longer Delay to ensure node has time to reset
    sendNMT(i, NMT_Start);                 // Send NMT commands to all motor drivers to switch to the Operational state
    delay(1000);                           // Delay to ensure command is processed
  }

  for (int i = THUMB; i <= PINKIE; ++i) {
    // Transition to "Ready to Switch On"
    resetMotor(i, PROFILE_POSITION_MODE);  // prepare all motors, putting them in Profile Position Mode

    // Verify if motor is in the 'Operation enabled' state
    if (isMotorReady(readStatusword(i))) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.println(" is ready!");
    } else {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.println(" is not ready or has an error.");
    }
  }

  // Complete Homing process for each one of the motors
  for (int i = THUMB; i <= PINKIE; ++i) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.println(" is beginning Homing Sequence.");
    resetMotor(i, PROFILE_POSITION_MODE);
    homingSequence(i);  // run the Homing Sequence for this motor
    //homingSequenceFast(i);
    Serial.println("Homing sequence complete!");
    motorMaximumPoint[i] = motorZeroPoint[i] - motorTravel;  // enable the maximum point for each motor, direction is negative
    Serial.print("Max position for Motor ");
    Serial.print(i);
    Serial.print(" set to ");
    Serial.println(motorMaximumPoint[i]);
    setMaximalFollowingError(i, 150000);              // set a very large following error so that
    resetMotor(i, CYCLIC_SYNCHRONOUS_POSITION_MODE);  // get the motor ready for CSP mode
  }

  // This loop runs through the state machine so each motor is functional in CSP mode
  for (int i = THUMB; i <= PINKIE; ++i) {
    readModeOfOperationSDO(i);
    delay(50);
    readStatusword(i);
    delay(50);
    writeControlwordSDO(i, SHUTDOWN);  // Transitions to "Ready To Switch On" state
    delay(20);
    writeControlwordSDO(i, SWITCH_ON);  // Transitions to "Switched On" state
    delay(20);
    writeControlwordSDO(i, ENABLE_OPERATION);  // Transitions to "Operation Enabled" state
    delay(20);
    writeTargetPositionPDO(i, motorMaximumPoint[i] + 5000);
    delay(50);
    readStatusword(i);
    delay(50);
    Serial.print("Target Position: ");
    Serial.println(readTargetPositionSDO(i));
    delay(50);
    writeTargetPositionSDO(i, motorMaximumPoint[i]);
    delay(50);
    readStatusword(i);
    delay(50);
    Serial.print("Target Position: ");
    Serial.println(readTargetPositionSDO(i));
    delay(50);
  }

}  // Setup complete

void loop() {
  static unsigned long lastSyncTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSyncTime >= syncPeriod) {
    lastSyncTime = currentTime;  // Update the last sync time

    // Create a CAN message for the SYNC
    CAN_message_t syncMsg;
    syncMsg.id = SYNC_COB_ID;
    syncMsg.len = 0;  // SYNC messages have no data

    // Send the SYNC message
    if (!Can1.write(syncMsg)) {  // if the SYNC fails
      Serial.println("SYNC failed.");
    }
  }

  cycleMotorPositions();
}

void cycleMotorPositions() {
  static unsigned long lastActionTime = 0;
  static uint8_t currentMotor = 1;
  static bool moveToPositionB = false;

  unsigned long currentTime = millis();

  // Check if it's time to actuate the next motor
  if (currentTime - lastActionTime >= 200) {
    lastActionTime = currentTime;  // Update the last action time
    // Determine the target position based on the toggle state
    int32_t targetPosition = moveToPositionB ?  motorMaximumPoint[currentMotor] + 2000 : motorZeroPoint[currentMotor] - 2000;
    // Actuate the current motor
    //writeTargetPositionPDO(currentMotor, targetPosition);
    writeTargetPositionSDO(currentMotor, targetPosition);

    // Prepare for the next motor
    if (++currentMotor > 5) {
      currentMotor = 1;                    // Reset to the first motor
      moveToPositionB = !moveToPositionB;  // Toggle between Position A and B
    }
  }
}

// Function to read current position of motor through SDO
int32_t readMotorPositionSDO(uint8_t nodeID) {
  uint8_t sdoReadCommand[8] = {
    SDO_READ_COMMAND,
    (uint8_t)(SDO_READ_POSITION_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(SDO_READ_POSITION_INDEX >> 8),    // Index HighByte
    SDO_READ_POSITION_SUBINDEX,                 // Subindex
    0x00, 0x00, 0x00, 0x00                      // Rest are reserved
  };
  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;  // Add the Node-ID
  outMsg.len = 8;                                  // Data is always 8 bytes
  memcpy(outMsg.buf, sdoReadCommand, 8);           // Copy the SDO command

  // Send the SDO read command
  if (!Can1.write(outMsg)) {
    Serial.print("SDO read position request FAILED for motor driver with Node-ID: ");
    Serial.println(nodeID);
  }

  // Now waiting for the response
  CAN_message_t inMsg;
  unsigned long timeout = millis() + 1000;  // set 1sec timeout

  while (millis() < timeout) {
    if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
      // Check if the received message is the SDO response
      if (inMsg.buf[1] == (uint8_t)(SDO_READ_POSITION_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(SDO_READ_POSITION_INDEX >> 8)) {
        // Assuming the data is 32-bit, construct the position from the response bytes
        int32_t position = inMsg.buf[4] | (inMsg.buf[5] << 8) | (inMsg.buf[6] << 16) | (inMsg.buf[7] << 24);
        return position;  // Return the position value
      }
    }
  }
  // If this point is ever reached, the response was not received in time
  Serial.print("Response from Node-ID ");
  Serial.print(nodeID);
  Serial.println(" was not received in time.");
  return errorValue;  // Return an error value of 200000 to indicate no position was received
}

// Function to read current position of motor through PDO
int32_t readMotorPositionPDO(uint8_t nodeID) {
  // Make sure a valid motor was selected
  if (nodeID < 1 || nodeID > 5) {
    Serial.println("Only select motor from 1 to 5 inclusive");
    return errorValue;
  }

  // Assign the correct COB-ID
  uint32_t motorPositionCOBID = 0x40000380 + nodeID;
  CAN_message_t msg;
  unsigned long startTime = millis();

  // Check the value within timeout
  while (millis() - startTime < timeout) {
    if (Can1.read(msg)) {
      if (msg.id == motorPositionCOBID) {
        return parseMotorPosition(msg);  // parse the actual motor position and return it
      }
    }
  }

  return errorValue;
}

// Minor function called upon in readMotorPositionPDO to actually parse the exact integer position
int32_t parseMotorPosition(const CAN_message_t& msg) {  // signed 32-bit integer because data length is 4 bytes and motor positions can be positive or negative
  // Check that message payload is at least 4 bytes long
  if (msg.len < 4) {
    Serial.println("Not enough data in this message");
    return errorValue;
  }

  // Parse the position which is in the first 4 bytes of the payload
  int32_t position = (int32_t)((uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8) | ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24));

  return position;
}

// Function to write a target position to a motor through PDO
bool writeTargetPositionPDO(uint8_t nodeID, int32_t targetPosition) {
  if (nodeID < 1 || nodeID > 5) {
    Serial.println("Invalid Motor ID. Only select motor from 1 to 5 inclusive");
    return false;
  }

  // Assign the correct COB-ID
  uint32_t motorTargetPosCOBID = 0x00000400 + nodeID;

  // Create a new CAN message
  CAN_message_t msg;
  msg.id = motorTargetPosCOBID;
  msg.len = 4;

  msg.buf[0] = targetPosition & 0xFF;
  msg.buf[1] = (targetPosition >> 8) & 0xFF;
  msg.buf[2] = (targetPosition >> 16) & 0xFF;
  msg.buf[3] = (targetPosition >> 24) & 0xFF;

  // Try to send the CAN message
  if (Can1.write(msg)) {
    Serial.print("Target position PDO sent to node ");
    Serial.println(nodeID);
    return true;
  } else {
    Serial.print("Failed to send target position PDO to node ");
    Serial.println(nodeID);
    return false;
  }
}

// Function to write a target position to a motor through SDO
bool writeTargetPositionSDO(uint8_t nodeID, int32_t targetPosition) {
  if (nodeID < 1 || nodeID > 5) {
    Serial.println("Invalid Motor ID. Only select motor from 1 to 5 inclusive");
    return false;
  }

  uint8_t sdoWriteCommand[8] = {
    //SDO_WRITE_POSITION_COMMAND,
    0x23,
    (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX >> 8),    // Index HighByte
    SDO_WRITE_TARGET_POSITION_SUBINDEX,                 // Subindex
    (uint8_t)(targetPosition & 0xFF),                   // Target position LowByte
    (uint8_t)((targetPosition >> 8) & 0xFF),            // Target position MidLowByte
    (uint8_t)((targetPosition >> 16) & 0xFF),           // Target position MidHighByte
    (uint8_t)((targetPosition >> 24) & 0xFF)            // Target position HighByte
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;
  outMsg.len = 8;
  memcpy(outMsg.buf, sdoWriteCommand, 8);

  // Send the SDO write command
  if (Can1.write(outMsg)) {
    Serial.print("Sent SDO write request to motor driver with Node-ID: ");
    Serial.print(nodeID);
    Serial.print(" to move to position: ");
    Serial.println(targetPosition);

    // Wait for the SDO write confirmation
    CAN_message_t inMsg;
    unsigned long timeout = millis() + 1000;  // Set a 1-second timeout for the response

    while (millis() < timeout) {
      if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
        // Check if the received message is an SDO write confirmation
        if (inMsg.buf[0] == SDO_WRITE_CONFIRMATION_COMMAND && inMsg.buf[1] == (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX >> 8)) {
          Serial.println("SDO write confirmation received.");
          return true;  // SDO write was successful
        }
      }
    }
    Serial.println("SDO write confirmation not received.");
    return false;  // SDO write was not confirmed
  } else {
    Serial.print("Failed to send target position SDO to node ");
    Serial.println(nodeID);
    return false;
  }
}

bool setMaximalFollowingError(uint8_t nodeID, uint32_t followingError) {
  if (nodeID < 1 || nodeID > 5) {
    Serial.println("Invalid Motor ID. Only select motor from 1 to 5 inclusive");
    return false;
  }

  uint8_t sdoWriteCommand[8] = {
    //SDO_WRITE_POSITION_COMMAND,
    0x23,
    (uint8_t)(MAXIMAL_FOLLOWING_ERROR_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(MAXIMAL_FOLLOWING_ERROR_INDEX >> 8),    // Index HighByte
    MAXIMAL_FOLLOWING_ERROR_SUBINDEX,                 // Subindex
    (uint8_t)(followingError & 0xFF),                 //  LowByte
    (uint8_t)((followingError >> 8) & 0xFF),          // MidLowByte
    (uint8_t)((followingError >> 16) & 0xFF),         // MidHighByte
    (uint8_t)((followingError >> 24) & 0xFF)          // HighByte
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;
  outMsg.len = 8;
  memcpy(outMsg.buf, sdoWriteCommand, 8);

  // Send the SDO write command
  if (Can1.write(outMsg)) {
    Serial.print("Sent maxFollowingError write request to Node-ID: ");
    Serial.print(nodeID);
    Serial.print(" to error of: ");
    Serial.println(followingError);

    // Wait for the SDO write confirmation
    CAN_message_t inMsg;
    unsigned long timeout = millis() + 1000;  // Set a 1-second timeout for the response

    while (millis() < timeout) {
      if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
        // Check if the received message is an SDO write confirmation
        if (inMsg.buf[0] == SDO_WRITE_CONFIRMATION_COMMAND && inMsg.buf[1] == (uint8_t)(MAXIMAL_FOLLOWING_ERROR_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(MAXIMAL_FOLLOWING_ERROR_INDEX >> 8)) {
          Serial.println("SDO write confirmation received.");
          return true;  // SDO write was successful
        }
      }
    }
    Serial.println("SDO write confirmation not received.");
    return false;  // SDO write was not confirmed
  } else {
    Serial.print("Failed to set maxFollowingError to node ");
    Serial.println(nodeID);
    return false;
  }
}

bool writeControlWordPDO(uint8_t nodeID, uint16_t Controlword) {
  if (nodeID < 1 || nodeID > 5) {
    Serial.println("Invalid Motor ID. Only select motor from 1 to 5 inclusive");
    return false;
  }

  // Assign the correct COB-ID
  uint32_t ControlwordCOBID = 0x00000200 + nodeID;

  // Create a new CAN message
  CAN_message_t msg;
  msg.id = ControlwordCOBID;
  msg.len = 2;  // 16-bit data
  msg.buf[0] = Controlword & 0xFF;
  msg.buf[1] = (Controlword >> 8) & 0xFF;

  // Try to send the CAN message
  if (Can1.write(msg)) {
    Serial.print("controlword PDO sent to node ");
    Serial.println(nodeID);
    return true;
  } else {
    Serial.print("Failed to send controlword PDO to node ");
    Serial.println(nodeID);
    return false;
  }
}

int32_t readTargetPositionSDO(uint8_t nodeID) {
  uint8_t sdoReadCommand[8] = {
    SDO_READ_COMMAND,
    (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX & 0xFF),
    (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX >> 8),
    SDO_WRITE_TARGET_POSITION_SUBINDEX,
    0x00, 0x00, 0x00, 0x00
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;  // Add the Node-ID
  outMsg.len = 8;                                  // Data length for CANopen SDO is always 8 bytes
  memcpy(outMsg.buf, sdoReadCommand, 8);           // Copy the SDO command into the message buffer

  // Send the SDO read command
  if (!Can1.write(outMsg)) {
    Serial.print("SDO read target position request FAILED for motor driver with Node-ID: ");
    Serial.println(nodeID);
  }

  // Now waiting for the response
  CAN_message_t inMsg;
  unsigned long timeout = millis() + 1000;  // Set a 1-second timeout for the response

  while (millis() < timeout) {
    if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
      // Check if the received message is the SDO response for the target position
      if (inMsg.buf[1] == (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(SDO_WRITE_TARGET_POSITION_INDEX >> 8)) {
        // Construct the target position from the response bytes (assuming the data is a 32-bit integer)
        int32_t targetPosition = inMsg.buf[4] | (inMsg.buf[5] << 8) | (inMsg.buf[6] << 16) | (inMsg.buf[7] << 24);
        return targetPosition;  // Return the target position value
      }
    }
  }

  // If this point is ever reached, the response was not received in time
  Serial.print("Target Position Response from Node-ID ");
  Serial.print(nodeID);
  Serial.println(" was not received in time.");
  return errorValue;  // Return an error value to indicate no position was received
}


int8_t readModeOfOperationSDO(uint8_t nodeID) {
  uint8_t sdoReadCommand[8] = {
    SDO_READ_COMMAND,
    (uint8_t)(MODES_OF_OPERATION_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(MODES_OF_OPERATION_INDEX >> 8),    // Index HighByte
    MODES_OF_OPERATION_SUBINDEX,                 // Subindex
    0x00, 0x00, 0x00, 0x00                       // Rest are reserved
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;  // Add the Node-ID
  outMsg.len = 8;
  memcpy(outMsg.buf, sdoReadCommand, 8);  // Copy the SDO command

  // Send the SDO read command
  Can1.write(outMsg);
  Serial.print("Sent SDO read mode of operation request to motor driver with Node-ID: ");
  Serial.println(nodeID);

  // Now waiting for the response
  CAN_message_t inMsg;
  unsigned long timeout = millis() + 1000;  // set 1sec timeout for response

  while (millis() < timeout) {
    if (Can1.read(inMsg) && inMsg.id == (SDO_SERVER_TO_CLIENT_BASE + nodeID)) {
      // Check if the received message is the SDO response for Modes of Operation
      if (inMsg.buf[1] == (uint8_t)(MODES_OF_OPERATION_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(MODES_OF_OPERATION_INDEX >> 8)) {
        // Assuming the data is 8-bit (int8_t), extract the mode of operation from the response bytes
        int8_t modeOfOperation = inMsg.buf[4];  // Byte 4 contains the mode of operation value
        Serial.print("Received mode of operation from Node-ID ");
        Serial.print(nodeID);
        Serial.print(": ");
        Serial.println(modeOfOperation);
        return modeOfOperation;  // Return the mode of operation value
      }
    }
  }

  // If this point is reached, the response was not received in time
  Serial.print("Response from Node-ID ");
  Serial.print(nodeID);
  Serial.println(" was not received in time.");
  return errorValue;  // Return an error value
}

// Function to clear errors on motors
void cleanErrorState(uint8_t nodeID) {
  writeControlwordSDO(nodeID, FAULT_RESET);   // Set bit 7 in the Controlword to reset the fault
  delay(100);                                 // Small delay to ensure the command is processed
  resetMotor(nodeID, PROFILE_POSITION_MODE);  // put the motor back into Profile Position Mode
  delay(100);                                 // Small delay to ensure the command is processed

  // Confirm that the error is cleared by checking the Statusword
  uint16_t statusword = readStatusword(nodeID);
  if (!(statusword & 0x0008)) {  // Bit 3 (0x08) is the fault bit in Statusword
    Serial.println("Fault reset successful!");
  } else {
    Serial.println("Fault reset failed. Fault still present.");
  }
}

// Function used during Homing sequence to handle the error at the base of the travel
bool handleFollowingError(uint8_t nodeID, int32_t currentPosition) {
  // Check the drive's status to determine if a following error has occurred
  uint16_t statusword = readStatusword(nodeID);
  if (statusword & FAULT_MASK) {
    Serial.println("Following error detected, handling error.");

    cleanErrorState(nodeID);  // Attempt to clear the error

    // Read the status word again to check if the error has been cleared
    statusword = readStatusword(nodeID);
    if (statusword & FAULT_MASK) {
      Serial.println("Failed to clear the following error.");
      return false;  // the error was not handled correctly
    }

    motorZeroPoint[nodeID] = currentPosition - 2000;  // save the Zero (Home) point of the motor as 2000 increments from total travel

    if (!setHomePosition(nodeID, motorZeroPoint[nodeID])) {  // Then, update 2000 increments in from the current position to be the new Zero (Home) Position
      return false;                                          // tell the system that the reset of home was unsuccessful
    }
    moveToPosition(nodeID, motorZeroPoint[nodeID]);  // move the motor to its new Zero (Home) point
    Serial.print("Zero (Home) for Node ");
    Serial.print(nodeID);
    Serial.print(" set at: ");
    Serial.println(motorZeroPoint[nodeID]);
    return true;  // if it all went fine, return success
  } else {        // if no error was detected
    Serial.println("No following error detected.");
    return true;
  }
}

// Function to set the Home position
bool setHomePosition(uint8_t nodeID, int32_t desiredHomePosition) {
  uint8_t sdoWriteCommand[8] = {
    0x23,                                           // Command specifier for writing 4 bytes (0x23 for 32-bit, 0x2B for 16-bit, 0x2F for 8-bit)
    (uint8_t)(HOME_POSITION_INDEX & 0xFF),          // Index LowByte
    (uint8_t)(HOME_POSITION_INDEX >> 8),            // Index HighByte
    HOME_POSITION_SUBINDEX,                         // Subindex
    (uint8_t)(desiredHomePosition & 0xFF),          // Data LowByte
    (uint8_t)((desiredHomePosition >> 8) & 0xFF),   // Data Low MidByte
    (uint8_t)((desiredHomePosition >> 16) & 0xFF),  // Data High MidByte
    (uint8_t)(desiredHomePosition >> 24)            // Data HighByte
  };

  CAN_message_t outMsg;
  outMsg.id = SDO_CLIENT_TO_SERVER_BASE + nodeID;  // Add the node ID to the base Rx SDO ID
  outMsg.len = 8;
  memcpy(outMsg.buf, sdoWriteCommand, 8);

  if (Can1.write(outMsg)) {
    Serial.print("Home position set for motor with Node-ID: ");
    Serial.println(nodeID);
    return true;  // Success
  } else {
    Serial.print("Failed to set home position for motor with Node-ID: ");
    Serial.println(nodeID);
    return false;  // Failure
  }
}

// The primary Homing Sequence per motor
void homingSequence(uint8_t nodeID) {
  // Set the correct mode, in this case Profile Position Mode makes the most sense.
  // setModeOfOperation(nodeID, PROFILE_POSITION_MODE);
  // Serial.print("PROFILE POSITION MODE requested on Motor ");
  // Serial.println(nodeID);
  // delay(500);                                        // Short delay to ensure motor is in correct mode
  int8_t currentMode = readModeOfOperationSDO(nodeID);  // read the current mode of operation
  delay(500);
  uint16_t statuswordCurrent = readStatusword(nodeID);

  // Command the motor to move slowly in the positive direction
  setMaximalFollowingError(nodeID, 102);  // set the initial following error at 10% of the encoder resolution
  int32_t currentPosition = readMotorPositionSDO(nodeID);
  Serial.print("Start Position: ");
  Serial.println(currentPosition);
  while (!(readStatusword(nodeID) & FAULT_MASK)) {              // while there are no faults, keep repeating this process
    moveToPosition(nodeID, currentPosition + homingIncrement);  // move the motor to next position
    currentPosition += homingIncrement;                         // update the currentPosition variable
    Serial.print("Current Position: ");
    Serial.println(currentPosition);
    Serial.print("True Position: ");
    Serial.println(readMotorPositionSDO(nodeID));
    writeControlwordSDO(nodeID, NEW_POSITION);  // Prepare the system to accept a new position
    delay(200);
  };

  // This code is reached once the fault takes place
  handleFollowingError(nodeID, readMotorPositionSDO(nodeID));  // deal with the fault by clearing it, and assigning a new Home position
}

// An experimental very fast Homing Sequence I could never quite get working, the idea was to give it an impossibly far position to go to in a single movement
void homingSequenceFast(uint8_t nodeID) {
  int8_t currentMode = readModeOfOperationSDO(nodeID);  // read the current mode of operation
  delay(500);
  uint16_t statuswordCurrent = readStatusword(nodeID);
  int32_t currentPosition = readMotorPositionSDO(nodeID);
  Serial.print("Start Position: ");
  Serial.println(currentPosition);
  while (!(readStatusword(nodeID) & FAULT_MASK)) {
    moveToPosition(nodeID, 140000);  // move impossibly far in the positive direction
    Serial.print("Current Position: ");
    Serial.println(currentPosition);
    Serial.print("True Position: ");
    Serial.println(readMotorPositionSDO(nodeID));
    delay(3000);
  }
  // This code is reached once the fault takes place
  //handleFollowingError(nodeID, currentPosition);  // deal with the fault by clearing it, and assigning a new Home position
  handleFollowingError(nodeID, readMotorPositionSDO(nodeID));  // deal with the fault by clearing it, and assigning a new Home position
}

// SDO Function to actually move the motor into position, PDO equivalent doesn't exist because Cyclic Synchronous Position Mode doesn't need manual updates to move
void moveToPosition(uint8_t nodeID, uint32_t position) {

  writeTargetPositionSDO(nodeID, position);  // write the desired target position to the motor

  int32_t targetPosition = readTargetPositionSDO(nodeID);  // read the target position

  // Read the current status of the motor to determine the necessary Controlword command
  uint16_t status = readStatusword(nodeID);

  // Actually send the control word to the motor, this should make the motor move
  writeControlwordSDO(nodeID, ABSOLUTE_POSITION_IMMEDIATE);
  delay(50);
}


///// Check if used or not

uint16_t readErrorCode(uint8_t nodeID) {
  // Prepare SDO upload request to read error code from object 0x603F
  uint8_t sdoReadCommand[8] = {
    0x40,                                // SDO command specifier for reading 4 bytes
    (uint8_t)(ERROR_CODE_INDEX & 0xFF),  // Index LowByte
    (uint8_t)(ERROR_CODE_INDEX >> 8),    // Index HighByte
    ERROR_CODE_SUBINDEX,                 // Subindex
    0x00, 0x00, 0x00, 0x00               // The rest are zeros for SDO upload request
  };

  CAN_message_t outMsg;
  outMsg.id = 0x600 + nodeID;  // SDO client-to-server COB-ID for specified Node-ID
  outMsg.len = 8;
  memcpy(outMsg.buf, sdoReadCommand, 8);

  // Send the SDO read command
  if (Can1.write(outMsg)) {
    // Now wait for the response from the motor drive
    CAN_message_t inMsg;
    if (waitForErrorCodeResponse(nodeID, inMsg)) {
      // Extract the error code from the response message
      uint16_t errorCode = inMsg.buf[4] | (inMsg.buf[5] << 8);
      return errorCode;
    }
  }
  return 0xFFFF;  // Return an invalid error code if read failed
}

bool waitForErrorCodeResponse(uint8_t nodeID, CAN_message_t& inMsg) {
  // Wait for SDO response with a timeout
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {  // 1-second timeout
    if (Can1.read(inMsg) && inMsg.id == (0x580 + nodeID)) {
      // Check if this is the SDO response for the error code
      if (inMsg.buf[1] == (uint8_t)(ERROR_CODE_INDEX & 0xFF) && inMsg.buf[2] == (uint8_t)(ERROR_CODE_INDEX >> 8) && inMsg.buf[3] == ERROR_CODE_SUBINDEX) {
        return true;  // Got the response
      }
    }
  }
  return false;  // Timeout reached, no response
}
