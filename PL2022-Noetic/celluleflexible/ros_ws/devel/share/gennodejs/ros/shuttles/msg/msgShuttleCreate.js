// Auto-generated. Do not edit!

// (in-package shuttles.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class msgShuttleCreate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.destination = null;
      this.product = null;
      this.handle = null;
      this.handlePlatform = null;
      this.zone = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('destination')) {
        this.destination = initObj.destination
      }
      else {
        this.destination = 0;
      }
      if (initObj.hasOwnProperty('product')) {
        this.product = initObj.product
      }
      else {
        this.product = 0;
      }
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = 0;
      }
      if (initObj.hasOwnProperty('handlePlatform')) {
        this.handlePlatform = initObj.handlePlatform
      }
      else {
        this.handlePlatform = 0;
      }
      if (initObj.hasOwnProperty('zone')) {
        this.zone = initObj.zone
      }
      else {
        this.zone = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgShuttleCreate
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [destination]
    bufferOffset = _serializer.int32(obj.destination, buffer, bufferOffset);
    // Serialize message field [product]
    bufferOffset = _serializer.int32(obj.product, buffer, bufferOffset);
    // Serialize message field [handle]
    bufferOffset = _serializer.int32(obj.handle, buffer, bufferOffset);
    // Serialize message field [handlePlatform]
    bufferOffset = _serializer.int32(obj.handlePlatform, buffer, bufferOffset);
    // Serialize message field [zone]
    bufferOffset = _serializer.int32(obj.zone, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgShuttleCreate
    let len;
    let data = new msgShuttleCreate(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [destination]
    data.destination = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [product]
    data.product = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [handle]
    data.handle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [handlePlatform]
    data.handlePlatform = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [zone]
    data.zone = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'shuttles/msgShuttleCreate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bbe2beba2a1f997c0e8c09a3d7eb1f7a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    int32 destination
    int32 product
    int32 handle
    int32 handlePlatform
    int32 zone
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgShuttleCreate(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.destination !== undefined) {
      resolved.destination = msg.destination;
    }
    else {
      resolved.destination = 0
    }

    if (msg.product !== undefined) {
      resolved.product = msg.product;
    }
    else {
      resolved.product = 0
    }

    if (msg.handle !== undefined) {
      resolved.handle = msg.handle;
    }
    else {
      resolved.handle = 0
    }

    if (msg.handlePlatform !== undefined) {
      resolved.handlePlatform = msg.handlePlatform;
    }
    else {
      resolved.handlePlatform = 0
    }

    if (msg.zone !== undefined) {
      resolved.zone = msg.zone;
    }
    else {
      resolved.zone = 0
    }

    return resolved;
    }
};

module.exports = msgShuttleCreate;
