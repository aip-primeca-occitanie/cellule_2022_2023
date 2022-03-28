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

class DeplacerPieceMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_robot = null;
      this.positionA = null;
      this.positionB = null;
    }
    else {
      if (initObj.hasOwnProperty('num_robot')) {
        this.num_robot = initObj.num_robot
      }
      else {
        this.num_robot = 0;
      }
      if (initObj.hasOwnProperty('positionA')) {
        this.positionA = initObj.positionA
      }
      else {
        this.positionA = 0;
      }
      if (initObj.hasOwnProperty('positionB')) {
        this.positionB = initObj.positionB
      }
      else {
        this.positionB = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeplacerPieceMsg
    // Serialize message field [num_robot]
    bufferOffset = _serializer.int32(obj.num_robot, buffer, bufferOffset);
    // Serialize message field [positionA]
    bufferOffset = _serializer.int32(obj.positionA, buffer, bufferOffset);
    // Serialize message field [positionB]
    bufferOffset = _serializer.int32(obj.positionB, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeplacerPieceMsg
    let len;
    let data = new DeplacerPieceMsg(null);
    // Deserialize message field [num_robot]
    data.num_robot = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [positionA]
    data.positionA = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [positionB]
    data.positionB = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'commande_locale/DeplacerPieceMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3590f7e74b621fec7039b4d9aecf39ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 num_robot
    int32 positionA
    int32 positionB
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeplacerPieceMsg(null);
    if (msg.num_robot !== undefined) {
      resolved.num_robot = msg.num_robot;
    }
    else {
      resolved.num_robot = 0
    }

    if (msg.positionA !== undefined) {
      resolved.positionA = msg.positionA;
    }
    else {
      resolved.positionA = 0
    }

    if (msg.positionB !== undefined) {
      resolved.positionB = msg.positionB;
    }
    else {
      resolved.positionB = 0
    }

    return resolved;
    }
};

module.exports = DeplacerPieceMsg;
