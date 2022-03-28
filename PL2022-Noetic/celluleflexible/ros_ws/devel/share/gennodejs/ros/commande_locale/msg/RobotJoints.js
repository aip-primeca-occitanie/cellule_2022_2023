// Auto-generated. Do not edit!

// (in-package commande_locale.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RobotJoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint1 = null;
      this.joint2 = null;
      this.joint3 = null;
      this.joint4 = null;
      this.joint5 = null;
      this.joint6 = null;
      this.joint7 = null;
      this.num_robot = null;
    }
    else {
      if (initObj.hasOwnProperty('joint1')) {
        this.joint1 = initObj.joint1
      }
      else {
        this.joint1 = 0;
      }
      if (initObj.hasOwnProperty('joint2')) {
        this.joint2 = initObj.joint2
      }
      else {
        this.joint2 = 0;
      }
      if (initObj.hasOwnProperty('joint3')) {
        this.joint3 = initObj.joint3
      }
      else {
        this.joint3 = 0;
      }
      if (initObj.hasOwnProperty('joint4')) {
        this.joint4 = initObj.joint4
      }
      else {
        this.joint4 = 0;
      }
      if (initObj.hasOwnProperty('joint5')) {
        this.joint5 = initObj.joint5
      }
      else {
        this.joint5 = 0;
      }
      if (initObj.hasOwnProperty('joint6')) {
        this.joint6 = initObj.joint6
      }
      else {
        this.joint6 = 0;
      }
      if (initObj.hasOwnProperty('joint7')) {
        this.joint7 = initObj.joint7
      }
      else {
        this.joint7 = 0;
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
    // Serializes a message object of type RobotJoints
    // Serialize message field [joint1]
    bufferOffset = _serializer.int32(obj.joint1, buffer, bufferOffset);
    // Serialize message field [joint2]
    bufferOffset = _serializer.int32(obj.joint2, buffer, bufferOffset);
    // Serialize message field [joint3]
    bufferOffset = _serializer.int32(obj.joint3, buffer, bufferOffset);
    // Serialize message field [joint4]
    bufferOffset = _serializer.int32(obj.joint4, buffer, bufferOffset);
    // Serialize message field [joint5]
    bufferOffset = _serializer.int32(obj.joint5, buffer, bufferOffset);
    // Serialize message field [joint6]
    bufferOffset = _serializer.int32(obj.joint6, buffer, bufferOffset);
    // Serialize message field [joint7]
    bufferOffset = _serializer.int32(obj.joint7, buffer, bufferOffset);
    // Serialize message field [num_robot]
    bufferOffset = _serializer.int32(obj.num_robot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotJoints
    let len;
    let data = new RobotJoints(null);
    // Deserialize message field [joint1]
    data.joint1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint2]
    data.joint2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint3]
    data.joint3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint4]
    data.joint4 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint5]
    data.joint5 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint6]
    data.joint6 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint7]
    data.joint7 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [num_robot]
    data.num_robot = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commande_locale/RobotJoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83d02ab5b13f8f1d23146613b3ae26a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 joint1
    int32 joint2
    int32 joint3
    int32 joint4
    int32 joint5
    int32 joint6
    int32 joint7
    int32 num_robot
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotJoints(null);
    if (msg.joint1 !== undefined) {
      resolved.joint1 = msg.joint1;
    }
    else {
      resolved.joint1 = 0
    }

    if (msg.joint2 !== undefined) {
      resolved.joint2 = msg.joint2;
    }
    else {
      resolved.joint2 = 0
    }

    if (msg.joint3 !== undefined) {
      resolved.joint3 = msg.joint3;
    }
    else {
      resolved.joint3 = 0
    }

    if (msg.joint4 !== undefined) {
      resolved.joint4 = msg.joint4;
    }
    else {
      resolved.joint4 = 0
    }

    if (msg.joint5 !== undefined) {
      resolved.joint5 = msg.joint5;
    }
    else {
      resolved.joint5 = 0
    }

    if (msg.joint6 !== undefined) {
      resolved.joint6 = msg.joint6;
    }
    else {
      resolved.joint6 = 0
    }

    if (msg.joint7 !== undefined) {
      resolved.joint7 = msg.joint7;
    }
    else {
      resolved.joint7 = 0
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

module.exports = RobotJoints;
