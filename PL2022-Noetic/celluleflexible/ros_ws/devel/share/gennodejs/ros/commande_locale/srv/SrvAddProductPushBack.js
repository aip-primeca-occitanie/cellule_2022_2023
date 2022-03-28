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

class SrvAddProductPushBackRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poste = null;
      this.produit = null;
    }
    else {
      if (initObj.hasOwnProperty('poste')) {
        this.poste = initObj.poste
      }
      else {
        this.poste = 0;
      }
      if (initObj.hasOwnProperty('produit')) {
        this.produit = initObj.produit
      }
      else {
        this.produit = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrvAddProductPushBackRequest
    // Serialize message field [poste]
    bufferOffset = _serializer.int32(obj.poste, buffer, bufferOffset);
    // Serialize message field [produit]
    bufferOffset = _serializer.int32(obj.produit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrvAddProductPushBackRequest
    let len;
    let data = new SrvAddProductPushBackRequest(null);
    // Deserialize message field [poste]
    data.poste = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [produit]
    data.produit = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'commande_locale/SrvAddProductPushBackRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3039ca7d9d039656971a53824ad295d1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 poste
    int32 produit
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SrvAddProductPushBackRequest(null);
    if (msg.poste !== undefined) {
      resolved.poste = msg.poste;
    }
    else {
      resolved.poste = 0
    }

    if (msg.produit !== undefined) {
      resolved.produit = msg.produit;
    }
    else {
      resolved.produit = 0
    }

    return resolved;
    }
};

class SrvAddProductPushBackResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SrvAddProductPushBackResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SrvAddProductPushBackResponse
    let len;
    let data = new SrvAddProductPushBackResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'commande_locale/SrvAddProductPushBackResponse';
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
    const resolved = new SrvAddProductPushBackResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SrvAddProductPushBackRequest,
  Response: SrvAddProductPushBackResponse,
  md5sum() { return '3039ca7d9d039656971a53824ad295d1'; },
  datatype() { return 'commande_locale/SrvAddProductPushBack'; }
};
