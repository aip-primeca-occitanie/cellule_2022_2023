// Auto-generated. Do not edit!

// (in-package schneider_103.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Retour_cellule_103Request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.memoire = null;
    }
    else {
      if (initObj.hasOwnProperty('memoire')) {
        this.memoire = initObj.memoire
      }
      else {
        this.memoire = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Retour_cellule_103Request
    // Serialize message field [memoire]
    bufferOffset = _serializer.int32(obj.memoire, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Retour_cellule_103Request
    let len;
    let data = new Retour_cellule_103Request(null);
    // Deserialize message field [memoire]
    data.memoire = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'schneider_103/Retour_cellule_103Request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd38585a9ace9d44e8aeed18cb0d33881';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 memoire
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Retour_cellule_103Request(null);
    if (msg.memoire !== undefined) {
      resolved.memoire = msg.memoire;
    }
    else {
      resolved.memoire = 0
    }

    return resolved;
    }
};

class Retour_cellule_103Response {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ST6 = null;
      this.ST7 = null;
      this.ST18 = null;
      this.ST19 = null;
      this.R3D = null;
      this.R4D = null;
      this.R9D = null;
      this.R10D = null;
      this.R3G = null;
      this.R4G = null;
      this.R9G = null;
      this.R10G = null;
      this.D3 = null;
      this.D4 = null;
      this.D9 = null;
      this.D10 = null;
      this.V3 = null;
      this.V4 = null;
      this.V9 = null;
      this.V10 = null;
      this.PS7 = null;
      this.PS18 = null;
      this.PS19 = null;
      this.D3D = null;
      this.D4D = null;
      this.D9D = null;
      this.D10D = null;
      this.D3G = null;
      this.D4G = null;
      this.D9G = null;
      this.D10G = null;
      this.CP3 = null;
      this.CP8 = null;
    }
    else {
      if (initObj.hasOwnProperty('ST6')) {
        this.ST6 = initObj.ST6
      }
      else {
        this.ST6 = false;
      }
      if (initObj.hasOwnProperty('ST7')) {
        this.ST7 = initObj.ST7
      }
      else {
        this.ST7 = false;
      }
      if (initObj.hasOwnProperty('ST18')) {
        this.ST18 = initObj.ST18
      }
      else {
        this.ST18 = false;
      }
      if (initObj.hasOwnProperty('ST19')) {
        this.ST19 = initObj.ST19
      }
      else {
        this.ST19 = false;
      }
      if (initObj.hasOwnProperty('R3D')) {
        this.R3D = initObj.R3D
      }
      else {
        this.R3D = false;
      }
      if (initObj.hasOwnProperty('R4D')) {
        this.R4D = initObj.R4D
      }
      else {
        this.R4D = false;
      }
      if (initObj.hasOwnProperty('R9D')) {
        this.R9D = initObj.R9D
      }
      else {
        this.R9D = false;
      }
      if (initObj.hasOwnProperty('R10D')) {
        this.R10D = initObj.R10D
      }
      else {
        this.R10D = false;
      }
      if (initObj.hasOwnProperty('R3G')) {
        this.R3G = initObj.R3G
      }
      else {
        this.R3G = false;
      }
      if (initObj.hasOwnProperty('R4G')) {
        this.R4G = initObj.R4G
      }
      else {
        this.R4G = false;
      }
      if (initObj.hasOwnProperty('R9G')) {
        this.R9G = initObj.R9G
      }
      else {
        this.R9G = false;
      }
      if (initObj.hasOwnProperty('R10G')) {
        this.R10G = initObj.R10G
      }
      else {
        this.R10G = false;
      }
      if (initObj.hasOwnProperty('D3')) {
        this.D3 = initObj.D3
      }
      else {
        this.D3 = false;
      }
      if (initObj.hasOwnProperty('D4')) {
        this.D4 = initObj.D4
      }
      else {
        this.D4 = false;
      }
      if (initObj.hasOwnProperty('D9')) {
        this.D9 = initObj.D9
      }
      else {
        this.D9 = false;
      }
      if (initObj.hasOwnProperty('D10')) {
        this.D10 = initObj.D10
      }
      else {
        this.D10 = false;
      }
      if (initObj.hasOwnProperty('V3')) {
        this.V3 = initObj.V3
      }
      else {
        this.V3 = false;
      }
      if (initObj.hasOwnProperty('V4')) {
        this.V4 = initObj.V4
      }
      else {
        this.V4 = false;
      }
      if (initObj.hasOwnProperty('V9')) {
        this.V9 = initObj.V9
      }
      else {
        this.V9 = false;
      }
      if (initObj.hasOwnProperty('V10')) {
        this.V10 = initObj.V10
      }
      else {
        this.V10 = false;
      }
      if (initObj.hasOwnProperty('PS7')) {
        this.PS7 = initObj.PS7
      }
      else {
        this.PS7 = false;
      }
      if (initObj.hasOwnProperty('PS18')) {
        this.PS18 = initObj.PS18
      }
      else {
        this.PS18 = false;
      }
      if (initObj.hasOwnProperty('PS19')) {
        this.PS19 = initObj.PS19
      }
      else {
        this.PS19 = false;
      }
      if (initObj.hasOwnProperty('D3D')) {
        this.D3D = initObj.D3D
      }
      else {
        this.D3D = false;
      }
      if (initObj.hasOwnProperty('D4D')) {
        this.D4D = initObj.D4D
      }
      else {
        this.D4D = false;
      }
      if (initObj.hasOwnProperty('D9D')) {
        this.D9D = initObj.D9D
      }
      else {
        this.D9D = false;
      }
      if (initObj.hasOwnProperty('D10D')) {
        this.D10D = initObj.D10D
      }
      else {
        this.D10D = false;
      }
      if (initObj.hasOwnProperty('D3G')) {
        this.D3G = initObj.D3G
      }
      else {
        this.D3G = false;
      }
      if (initObj.hasOwnProperty('D4G')) {
        this.D4G = initObj.D4G
      }
      else {
        this.D4G = false;
      }
      if (initObj.hasOwnProperty('D9G')) {
        this.D9G = initObj.D9G
      }
      else {
        this.D9G = false;
      }
      if (initObj.hasOwnProperty('D10G')) {
        this.D10G = initObj.D10G
      }
      else {
        this.D10G = false;
      }
      if (initObj.hasOwnProperty('CP3')) {
        this.CP3 = initObj.CP3
      }
      else {
        this.CP3 = false;
      }
      if (initObj.hasOwnProperty('CP8')) {
        this.CP8 = initObj.CP8
      }
      else {
        this.CP8 = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Retour_cellule_103Response
    // Serialize message field [ST6]
    bufferOffset = _serializer.bool(obj.ST6, buffer, bufferOffset);
    // Serialize message field [ST7]
    bufferOffset = _serializer.bool(obj.ST7, buffer, bufferOffset);
    // Serialize message field [ST18]
    bufferOffset = _serializer.bool(obj.ST18, buffer, bufferOffset);
    // Serialize message field [ST19]
    bufferOffset = _serializer.bool(obj.ST19, buffer, bufferOffset);
    // Serialize message field [R3D]
    bufferOffset = _serializer.bool(obj.R3D, buffer, bufferOffset);
    // Serialize message field [R4D]
    bufferOffset = _serializer.bool(obj.R4D, buffer, bufferOffset);
    // Serialize message field [R9D]
    bufferOffset = _serializer.bool(obj.R9D, buffer, bufferOffset);
    // Serialize message field [R10D]
    bufferOffset = _serializer.bool(obj.R10D, buffer, bufferOffset);
    // Serialize message field [R3G]
    bufferOffset = _serializer.bool(obj.R3G, buffer, bufferOffset);
    // Serialize message field [R4G]
    bufferOffset = _serializer.bool(obj.R4G, buffer, bufferOffset);
    // Serialize message field [R9G]
    bufferOffset = _serializer.bool(obj.R9G, buffer, bufferOffset);
    // Serialize message field [R10G]
    bufferOffset = _serializer.bool(obj.R10G, buffer, bufferOffset);
    // Serialize message field [D3]
    bufferOffset = _serializer.bool(obj.D3, buffer, bufferOffset);
    // Serialize message field [D4]
    bufferOffset = _serializer.bool(obj.D4, buffer, bufferOffset);
    // Serialize message field [D9]
    bufferOffset = _serializer.bool(obj.D9, buffer, bufferOffset);
    // Serialize message field [D10]
    bufferOffset = _serializer.bool(obj.D10, buffer, bufferOffset);
    // Serialize message field [V3]
    bufferOffset = _serializer.bool(obj.V3, buffer, bufferOffset);
    // Serialize message field [V4]
    bufferOffset = _serializer.bool(obj.V4, buffer, bufferOffset);
    // Serialize message field [V9]
    bufferOffset = _serializer.bool(obj.V9, buffer, bufferOffset);
    // Serialize message field [V10]
    bufferOffset = _serializer.bool(obj.V10, buffer, bufferOffset);
    // Serialize message field [PS7]
    bufferOffset = _serializer.bool(obj.PS7, buffer, bufferOffset);
    // Serialize message field [PS18]
    bufferOffset = _serializer.bool(obj.PS18, buffer, bufferOffset);
    // Serialize message field [PS19]
    bufferOffset = _serializer.bool(obj.PS19, buffer, bufferOffset);
    // Serialize message field [D3D]
    bufferOffset = _serializer.bool(obj.D3D, buffer, bufferOffset);
    // Serialize message field [D4D]
    bufferOffset = _serializer.bool(obj.D4D, buffer, bufferOffset);
    // Serialize message field [D9D]
    bufferOffset = _serializer.bool(obj.D9D, buffer, bufferOffset);
    // Serialize message field [D10D]
    bufferOffset = _serializer.bool(obj.D10D, buffer, bufferOffset);
    // Serialize message field [D3G]
    bufferOffset = _serializer.bool(obj.D3G, buffer, bufferOffset);
    // Serialize message field [D4G]
    bufferOffset = _serializer.bool(obj.D4G, buffer, bufferOffset);
    // Serialize message field [D9G]
    bufferOffset = _serializer.bool(obj.D9G, buffer, bufferOffset);
    // Serialize message field [D10G]
    bufferOffset = _serializer.bool(obj.D10G, buffer, bufferOffset);
    // Serialize message field [CP3]
    bufferOffset = _serializer.bool(obj.CP3, buffer, bufferOffset);
    // Serialize message field [CP8]
    bufferOffset = _serializer.bool(obj.CP8, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Retour_cellule_103Response
    let len;
    let data = new Retour_cellule_103Response(null);
    // Deserialize message field [ST6]
    data.ST6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST7]
    data.ST7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST18]
    data.ST18 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST19]
    data.ST19 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R3D]
    data.R3D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R4D]
    data.R4D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R9D]
    data.R9D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R10D]
    data.R10D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R3G]
    data.R3G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R4G]
    data.R4G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R9G]
    data.R9G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R10G]
    data.R10G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D3]
    data.D3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D4]
    data.D4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D9]
    data.D9 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D10]
    data.D10 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V3]
    data.V3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V4]
    data.V4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V9]
    data.V9 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V10]
    data.V10 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS7]
    data.PS7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS18]
    data.PS18 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS19]
    data.PS19 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D3D]
    data.D3D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D4D]
    data.D4D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D9D]
    data.D9D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D10D]
    data.D10D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D3G]
    data.D3G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D4G]
    data.D4G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D9G]
    data.D9G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D10G]
    data.D10G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CP3]
    data.CP3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CP8]
    data.CP8 = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 33;
  }

  static datatype() {
    // Returns string type for a service object
    return 'schneider_103/Retour_cellule_103Response';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d53ccdf0331758f5eea11c34a6186be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool ST6
    bool ST7
    bool ST18
    bool ST19
    bool R3D
    bool R4D
    bool R9D
    bool R10D
    bool R3G
    bool R4G
    bool R9G
    bool R10G
    bool D3
    bool D4
    bool D9
    bool D10
    bool V3
    bool V4
    bool V9
    bool V10
    bool PS7
    bool PS18
    bool PS19
    bool D3D
    bool D4D
    bool D9D
    bool D10D
    bool D3G
    bool D4G
    bool D9G
    bool D10G
    bool CP3
    bool CP8
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Retour_cellule_103Response(null);
    if (msg.ST6 !== undefined) {
      resolved.ST6 = msg.ST6;
    }
    else {
      resolved.ST6 = false
    }

    if (msg.ST7 !== undefined) {
      resolved.ST7 = msg.ST7;
    }
    else {
      resolved.ST7 = false
    }

    if (msg.ST18 !== undefined) {
      resolved.ST18 = msg.ST18;
    }
    else {
      resolved.ST18 = false
    }

    if (msg.ST19 !== undefined) {
      resolved.ST19 = msg.ST19;
    }
    else {
      resolved.ST19 = false
    }

    if (msg.R3D !== undefined) {
      resolved.R3D = msg.R3D;
    }
    else {
      resolved.R3D = false
    }

    if (msg.R4D !== undefined) {
      resolved.R4D = msg.R4D;
    }
    else {
      resolved.R4D = false
    }

    if (msg.R9D !== undefined) {
      resolved.R9D = msg.R9D;
    }
    else {
      resolved.R9D = false
    }

    if (msg.R10D !== undefined) {
      resolved.R10D = msg.R10D;
    }
    else {
      resolved.R10D = false
    }

    if (msg.R3G !== undefined) {
      resolved.R3G = msg.R3G;
    }
    else {
      resolved.R3G = false
    }

    if (msg.R4G !== undefined) {
      resolved.R4G = msg.R4G;
    }
    else {
      resolved.R4G = false
    }

    if (msg.R9G !== undefined) {
      resolved.R9G = msg.R9G;
    }
    else {
      resolved.R9G = false
    }

    if (msg.R10G !== undefined) {
      resolved.R10G = msg.R10G;
    }
    else {
      resolved.R10G = false
    }

    if (msg.D3 !== undefined) {
      resolved.D3 = msg.D3;
    }
    else {
      resolved.D3 = false
    }

    if (msg.D4 !== undefined) {
      resolved.D4 = msg.D4;
    }
    else {
      resolved.D4 = false
    }

    if (msg.D9 !== undefined) {
      resolved.D9 = msg.D9;
    }
    else {
      resolved.D9 = false
    }

    if (msg.D10 !== undefined) {
      resolved.D10 = msg.D10;
    }
    else {
      resolved.D10 = false
    }

    if (msg.V3 !== undefined) {
      resolved.V3 = msg.V3;
    }
    else {
      resolved.V3 = false
    }

    if (msg.V4 !== undefined) {
      resolved.V4 = msg.V4;
    }
    else {
      resolved.V4 = false
    }

    if (msg.V9 !== undefined) {
      resolved.V9 = msg.V9;
    }
    else {
      resolved.V9 = false
    }

    if (msg.V10 !== undefined) {
      resolved.V10 = msg.V10;
    }
    else {
      resolved.V10 = false
    }

    if (msg.PS7 !== undefined) {
      resolved.PS7 = msg.PS7;
    }
    else {
      resolved.PS7 = false
    }

    if (msg.PS18 !== undefined) {
      resolved.PS18 = msg.PS18;
    }
    else {
      resolved.PS18 = false
    }

    if (msg.PS19 !== undefined) {
      resolved.PS19 = msg.PS19;
    }
    else {
      resolved.PS19 = false
    }

    if (msg.D3D !== undefined) {
      resolved.D3D = msg.D3D;
    }
    else {
      resolved.D3D = false
    }

    if (msg.D4D !== undefined) {
      resolved.D4D = msg.D4D;
    }
    else {
      resolved.D4D = false
    }

    if (msg.D9D !== undefined) {
      resolved.D9D = msg.D9D;
    }
    else {
      resolved.D9D = false
    }

    if (msg.D10D !== undefined) {
      resolved.D10D = msg.D10D;
    }
    else {
      resolved.D10D = false
    }

    if (msg.D3G !== undefined) {
      resolved.D3G = msg.D3G;
    }
    else {
      resolved.D3G = false
    }

    if (msg.D4G !== undefined) {
      resolved.D4G = msg.D4G;
    }
    else {
      resolved.D4G = false
    }

    if (msg.D9G !== undefined) {
      resolved.D9G = msg.D9G;
    }
    else {
      resolved.D9G = false
    }

    if (msg.D10G !== undefined) {
      resolved.D10G = msg.D10G;
    }
    else {
      resolved.D10G = false
    }

    if (msg.CP3 !== undefined) {
      resolved.CP3 = msg.CP3;
    }
    else {
      resolved.CP3 = false
    }

    if (msg.CP8 !== undefined) {
      resolved.CP8 = msg.CP8;
    }
    else {
      resolved.CP8 = false
    }

    return resolved;
    }
};

module.exports = {
  Request: Retour_cellule_103Request,
  Response: Retour_cellule_103Response,
  md5sum() { return '144215a6005e994c4be2aa0a861077f5'; },
  datatype() { return 'schneider_103/Retour_cellule_103'; }
};
