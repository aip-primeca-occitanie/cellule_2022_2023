// Auto-generated. Do not edit!

// (in-package shuttles.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class shuttle_idRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot = null;
      this.position = null;
    }
    else {
      if (initObj.hasOwnProperty('robot')) {
        this.robot = initObj.robot
      }
      else {
        this.robot = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shuttle_idRequest
    // Serialize message field [robot]
    bufferOffset = _serializer.int32(obj.robot, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.int32(obj.position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shuttle_idRequest
    let len;
    let data = new shuttle_idRequest(null);
    // Deserialize message field [robot]
    data.robot = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'shuttles/shuttle_idRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7bc20435b1237c98bd1aa24e06515eb0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 robot
    int32 position
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shuttle_idRequest(null);
    if (msg.robot !== undefined) {
      resolved.robot = msg.robot;
    }
    else {
      resolved.robot = 0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    return resolved;
    }
};

class shuttle_idResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.IdShuttle = null;
    }
    else {
      if (initObj.hasOwnProperty('IdShuttle')) {
        this.IdShuttle = initObj.IdShuttle
      }
      else {
        this.IdShuttle = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type shuttle_idResponse
    // Serialize message field [IdShuttle]
    bufferOffset = _serializer.int32(obj.IdShuttle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type shuttle_idResponse
    let len;
    let data = new shuttle_idResponse(null);
    // Deserialize message field [IdShuttle]
    data.IdShuttle = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'shuttles/shuttle_idResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39f3cc90c3f781e09c842cf3e85b7aa9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 IdShuttle
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new shuttle_idResponse(null);
    if (msg.IdShuttle !== undefined) {
      resolved.IdShuttle = msg.IdShuttle;
    }
    else {
      resolved.IdShuttle = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: shuttle_idRequest,
  Response: shuttle_idResponse,
  md5sum() { return '4ce93559e67d465c0b309aaf61a7e04b'; },
  datatype() { return 'shuttles/shuttle_id'; }
};
