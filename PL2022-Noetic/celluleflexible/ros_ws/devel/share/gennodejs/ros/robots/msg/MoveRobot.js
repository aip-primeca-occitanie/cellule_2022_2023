// Auto-generated. Do not edit!

// (in-package robots.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MoveRobot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.bras = null;
      this.pince = null;
      this.num_robot = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
      if (initObj.hasOwnProperty('bras')) {
        this.bras = initObj.bras
      }
      else {
        this.bras = 0;
      }
      if (initObj.hasOwnProperty('pince')) {
        this.pince = initObj.pince
      }
      else {
        this.pince = 0;
      }
      if (initObj.hasOwnProperty('num_robot')) {
        this.num_robot = initObj.num_robot
      }
      else {
        this.num_robot = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveRobot
    // Serialize message field [position]
    bufferOffset = _serializer.int32(obj.position, buffer, bufferOffset);
    // Serialize message field [bras]
    bufferOffset = _serializer.int32(obj.bras, buffer, bufferOffset);
    // Serialize message field [pince]
    bufferOffset = _serializer.int32(obj.pince, buffer, bufferOffset);
    // Serialize message field [num_robot]
    bufferOffset = _serializer.int32(obj.num_robot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveRobot
    let len;
    let data = new MoveRobot(null);
    // Deserialize message field [position]
    data.position = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [bras]
    data.bras = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pince]
    data.pince = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_robot]
    data.num_robot = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robots/MoveRobot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a71d2b8fb832f571ecc6a220d76058b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 position
    int32 bras
    int32 pince
    int32 num_robot
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveRobot(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    if (msg.bras !== undefined) {
      resolved.bras = msg.bras;
    }
    else {
      resolved.bras = 0
    }

    if (msg.pince !== undefined) {
      resolved.pince = msg.pince;
    }
    else {
      resolved.pince = 0
    }

    if (msg.num_robot !== undefined) {
      resolved.num_robot = msg.num_robot;
    }
    else {
      resolved.num_robot = 0
    }

    return resolved;
    }
};

module.exports = MoveRobot;
