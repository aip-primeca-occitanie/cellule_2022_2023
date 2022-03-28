// Auto-generated. Do not edit!

// (in-package commande_locale.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SrvAddProductRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.choixPoste = null;
      this.choixProduit = null;
    }
    else {
      if (initObj.hasOwnProperty('choixPoste')) {
        this.choixPoste = initObj.choixPoste
      }
      else {
        this.choixPoste = 0;
      }
      if (initObj.hasOwnProperty('choixProduit')) {
        this.choixProduit = initObj.choixProduit
      }
      else {
        this.choixProduit = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrvAddProductRequest
    // Serialize message field [choixPoste]
    bufferOffset = _serializer.int32(obj.choixPoste, buffer, bufferOffset);
    // Serialize message field [choixProduit]
    bufferOffset = _serializer.int32(obj.choixProduit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrvAddProductRequest
    let len;
    let data = new SrvAddProductRequest(null);
    // Deserialize message field [choixPoste]
    data.choixPoste = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [choixProduit]
    data.choixProduit = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'commande_locale/SrvAddProductRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02830da78068d9cdc27b052881f589fb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 choixPoste
    int32 choixProduit
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SrvAddProductRequest(null);
    if (msg.choixPoste !== undefined) {
      resolved.choixPoste = msg.choixPoste;
    }
    else {
      resolved.choixPoste = 0
    }

    if (msg.choixProduit !== undefined) {
      resolved.choixProduit = msg.choixProduit;
    }
    else {
      resolved.choixProduit = 0
    }

    return resolved;
    }
};

class SrvAddProductResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrvAddProductResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrvAddProductResponse
    let len;
    let data = new SrvAddProductResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'commande_locale/SrvAddProductResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SrvAddProductResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SrvAddProductRequest,
  Response: SrvAddProductResponse,
  md5sum() { return '02830da78068d9cdc27b052881f589fb'; },
  datatype() { return 'commande_locale/SrvAddProduct'; }
};
