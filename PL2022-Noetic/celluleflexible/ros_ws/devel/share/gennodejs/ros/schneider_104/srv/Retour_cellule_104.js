// Auto-generated. Do not edit!

// (in-package schneider_104.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Retour_cellule_104Request {
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
    // Serializes a message object of type Retour_cellule_104Request
    // Serialize message field [memoire]
    bufferOffset = _serializer.int32(obj.memoire, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Retour_cellule_104Request
    let len;
    let data = new Retour_cellule_104Request(null);
    // Deserialize message field [memoire]
    data.memoire = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'schneider_104/Retour_cellule_104Request';
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
    const resolved = new Retour_cellule_104Request(null);
    if (msg.memoire !== undefined) {
      resolved.memoire = msg.memoire;
    }
    else {
      resolved.memoire = 0
    }

    return resolved;
    }
};

class Retour_cellule_104Response {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ST8 = null;
      this.ST9 = null;
      this.ST10 = null;
      this.ST11 = null;
      this.ST12 = null;
      this.ST13 = null;
      this.ST14 = null;
      this.ST15 = null;
      this.ST16 = null;
      this.ST17 = null;
      this.R5D = null;
      this.R6D = null;
      this.R7D = null;
      this.R8D = null;
      this.R5G = null;
      this.R6G = null;
      this.R7G = null;
      this.R8G = null;
      this.PI3 = null;
      this.PI4 = null;
      this.PI5 = null;
      this.PI6 = null;
      this.D5 = null;
      this.D6 = null;
      this.D7 = null;
      this.D8 = null;
      this.V5 = null;
      this.V6 = null;
      this.V7 = null;
      this.V8 = null;
      this.PS8 = null;
      this.PS9 = null;
      this.PS10 = null;
      this.PS11 = null;
      this.PS12 = null;
      this.PS13 = null;
      this.PS14 = null;
      this.PS15 = null;
      this.PS16 = null;
      this.PS17 = null;
      this.D5D = null;
      this.D6D = null;
      this.D7D = null;
      this.D8D = null;
      this.D5G = null;
      this.D6G = null;
      this.D7G = null;
      this.D8G = null;
      this.CPI3 = null;
      this.CPI4 = null;
      this.CPI5 = null;
      this.CPI6 = null;
      this.CP4 = null;
      this.CP5 = null;
      this.CP6 = null;
      this.CP7 = null;
    }
    else {
      if (initObj.hasOwnProperty('ST8')) {
        this.ST8 = initObj.ST8
      }
      else {
        this.ST8 = false;
      }
      if (initObj.hasOwnProperty('ST9')) {
        this.ST9 = initObj.ST9
      }
      else {
        this.ST9 = false;
      }
      if (initObj.hasOwnProperty('ST10')) {
        this.ST10 = initObj.ST10
      }
      else {
        this.ST10 = false;
      }
      if (initObj.hasOwnProperty('ST11')) {
        this.ST11 = initObj.ST11
      }
      else {
        this.ST11 = false;
      }
      if (initObj.hasOwnProperty('ST12')) {
        this.ST12 = initObj.ST12
      }
      else {
        this.ST12 = false;
      }
      if (initObj.hasOwnProperty('ST13')) {
        this.ST13 = initObj.ST13
      }
      else {
        this.ST13 = false;
      }
      if (initObj.hasOwnProperty('ST14')) {
        this.ST14 = initObj.ST14
      }
      else {
        this.ST14 = false;
      }
      if (initObj.hasOwnProperty('ST15')) {
        this.ST15 = initObj.ST15
      }
      else {
        this.ST15 = false;
      }
      if (initObj.hasOwnProperty('ST16')) {
        this.ST16 = initObj.ST16
      }
      else {
        this.ST16 = false;
      }
      if (initObj.hasOwnProperty('ST17')) {
        this.ST17 = initObj.ST17
      }
      else {
        this.ST17 = false;
      }
      if (initObj.hasOwnProperty('R5D')) {
        this.R5D = initObj.R5D
      }
      else {
        this.R5D = false;
      }
      if (initObj.hasOwnProperty('R6D')) {
        this.R6D = initObj.R6D
      }
      else {
        this.R6D = false;
      }
      if (initObj.hasOwnProperty('R7D')) {
        this.R7D = initObj.R7D
      }
      else {
        this.R7D = false;
      }
      if (initObj.hasOwnProperty('R8D')) {
        this.R8D = initObj.R8D
      }
      else {
        this.R8D = false;
      }
      if (initObj.hasOwnProperty('R5G')) {
        this.R5G = initObj.R5G
      }
      else {
        this.R5G = false;
      }
      if (initObj.hasOwnProperty('R6G')) {
        this.R6G = initObj.R6G
      }
      else {
        this.R6G = false;
      }
      if (initObj.hasOwnProperty('R7G')) {
        this.R7G = initObj.R7G
      }
      else {
        this.R7G = false;
      }
      if (initObj.hasOwnProperty('R8G')) {
        this.R8G = initObj.R8G
      }
      else {
        this.R8G = false;
      }
      if (initObj.hasOwnProperty('PI3')) {
        this.PI3 = initObj.PI3
      }
      else {
        this.PI3 = false;
      }
      if (initObj.hasOwnProperty('PI4')) {
        this.PI4 = initObj.PI4
      }
      else {
        this.PI4 = false;
      }
      if (initObj.hasOwnProperty('PI5')) {
        this.PI5 = initObj.PI5
      }
      else {
        this.PI5 = false;
      }
      if (initObj.hasOwnProperty('PI6')) {
        this.PI6 = initObj.PI6
      }
      else {
        this.PI6 = false;
      }
      if (initObj.hasOwnProperty('D5')) {
        this.D5 = initObj.D5
      }
      else {
        this.D5 = false;
      }
      if (initObj.hasOwnProperty('D6')) {
        this.D6 = initObj.D6
      }
      else {
        this.D6 = false;
      }
      if (initObj.hasOwnProperty('D7')) {
        this.D7 = initObj.D7
      }
      else {
        this.D7 = false;
      }
      if (initObj.hasOwnProperty('D8')) {
        this.D8 = initObj.D8
      }
      else {
        this.D8 = false;
      }
      if (initObj.hasOwnProperty('V5')) {
        this.V5 = initObj.V5
      }
      else {
        this.V5 = false;
      }
      if (initObj.hasOwnProperty('V6')) {
        this.V6 = initObj.V6
      }
      else {
        this.V6 = false;
      }
      if (initObj.hasOwnProperty('V7')) {
        this.V7 = initObj.V7
      }
      else {
        this.V7 = false;
      }
      if (initObj.hasOwnProperty('V8')) {
        this.V8 = initObj.V8
      }
      else {
        this.V8 = false;
      }
      if (initObj.hasOwnProperty('PS8')) {
        this.PS8 = initObj.PS8
      }
      else {
        this.PS8 = false;
      }
      if (initObj.hasOwnProperty('PS9')) {
        this.PS9 = initObj.PS9
      }
      else {
        this.PS9 = false;
      }
      if (initObj.hasOwnProperty('PS10')) {
        this.PS10 = initObj.PS10
      }
      else {
        this.PS10 = false;
      }
      if (initObj.hasOwnProperty('PS11')) {
        this.PS11 = initObj.PS11
      }
      else {
        this.PS11 = false;
      }
      if (initObj.hasOwnProperty('PS12')) {
        this.PS12 = initObj.PS12
      }
      else {
        this.PS12 = false;
      }
      if (initObj.hasOwnProperty('PS13')) {
        this.PS13 = initObj.PS13
      }
      else {
        this.PS13 = false;
      }
      if (initObj.hasOwnProperty('PS14')) {
        this.PS14 = initObj.PS14
      }
      else {
        this.PS14 = false;
      }
      if (initObj.hasOwnProperty('PS15')) {
        this.PS15 = initObj.PS15
      }
      else {
        this.PS15 = false;
      }
      if (initObj.hasOwnProperty('PS16')) {
        this.PS16 = initObj.PS16
      }
      else {
        this.PS16 = false;
      }
      if (initObj.hasOwnProperty('PS17')) {
        this.PS17 = initObj.PS17
      }
      else {
        this.PS17 = false;
      }
      if (initObj.hasOwnProperty('D5D')) {
        this.D5D = initObj.D5D
      }
      else {
        this.D5D = false;
      }
      if (initObj.hasOwnProperty('D6D')) {
        this.D6D = initObj.D6D
      }
      else {
        this.D6D = false;
      }
      if (initObj.hasOwnProperty('D7D')) {
        this.D7D = initObj.D7D
      }
      else {
        this.D7D = false;
      }
      if (initObj.hasOwnProperty('D8D')) {
        this.D8D = initObj.D8D
      }
      else {
        this.D8D = false;
      }
      if (initObj.hasOwnProperty('D5G')) {
        this.D5G = initObj.D5G
      }
      else {
        this.D5G = false;
      }
      if (initObj.hasOwnProperty('D6G')) {
        this.D6G = initObj.D6G
      }
      else {
        this.D6G = false;
      }
      if (initObj.hasOwnProperty('D7G')) {
        this.D7G = initObj.D7G
      }
      else {
        this.D7G = false;
      }
      if (initObj.hasOwnProperty('D8G')) {
        this.D8G = initObj.D8G
      }
      else {
        this.D8G = false;
      }
      if (initObj.hasOwnProperty('CPI3')) {
        this.CPI3 = initObj.CPI3
      }
      else {
        this.CPI3 = false;
      }
      if (initObj.hasOwnProperty('CPI4')) {
        this.CPI4 = initObj.CPI4
      }
      else {
        this.CPI4 = false;
      }
      if (initObj.hasOwnProperty('CPI5')) {
        this.CPI5 = initObj.CPI5
      }
      else {
        this.CPI5 = false;
      }
      if (initObj.hasOwnProperty('CPI6')) {
        this.CPI6 = initObj.CPI6
      }
      else {
        this.CPI6 = false;
      }
      if (initObj.hasOwnProperty('CP4')) {
        this.CP4 = initObj.CP4
      }
      else {
        this.CP4 = false;
      }
      if (initObj.hasOwnProperty('CP5')) {
        this.CP5 = initObj.CP5
      }
      else {
        this.CP5 = false;
      }
      if (initObj.hasOwnProperty('CP6')) {
        this.CP6 = initObj.CP6
      }
      else {
        this.CP6 = false;
      }
      if (initObj.hasOwnProperty('CP7')) {
        this.CP7 = initObj.CP7
      }
      else {
        this.CP7 = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Retour_cellule_104Response
    // Serialize message field [ST8]
    bufferOffset = _serializer.bool(obj.ST8, buffer, bufferOffset);
    // Serialize message field [ST9]
    bufferOffset = _serializer.bool(obj.ST9, buffer, bufferOffset);
    // Serialize message field [ST10]
    bufferOffset = _serializer.bool(obj.ST10, buffer, bufferOffset);
    // Serialize message field [ST11]
    bufferOffset = _serializer.bool(obj.ST11, buffer, bufferOffset);
    // Serialize message field [ST12]
    bufferOffset = _serializer.bool(obj.ST12, buffer, bufferOffset);
    // Serialize message field [ST13]
    bufferOffset = _serializer.bool(obj.ST13, buffer, bufferOffset);
    // Serialize message field [ST14]
    bufferOffset = _serializer.bool(obj.ST14, buffer, bufferOffset);
    // Serialize message field [ST15]
    bufferOffset = _serializer.bool(obj.ST15, buffer, bufferOffset);
    // Serialize message field [ST16]
    bufferOffset = _serializer.bool(obj.ST16, buffer, bufferOffset);
    // Serialize message field [ST17]
    bufferOffset = _serializer.bool(obj.ST17, buffer, bufferOffset);
    // Serialize message field [R5D]
    bufferOffset = _serializer.bool(obj.R5D, buffer, bufferOffset);
    // Serialize message field [R6D]
    bufferOffset = _serializer.bool(obj.R6D, buffer, bufferOffset);
    // Serialize message field [R7D]
    bufferOffset = _serializer.bool(obj.R7D, buffer, bufferOffset);
    // Serialize message field [R8D]
    bufferOffset = _serializer.bool(obj.R8D, buffer, bufferOffset);
    // Serialize message field [R5G]
    bufferOffset = _serializer.bool(obj.R5G, buffer, bufferOffset);
    // Serialize message field [R6G]
    bufferOffset = _serializer.bool(obj.R6G, buffer, bufferOffset);
    // Serialize message field [R7G]
    bufferOffset = _serializer.bool(obj.R7G, buffer, bufferOffset);
    // Serialize message field [R8G]
    bufferOffset = _serializer.bool(obj.R8G, buffer, bufferOffset);
    // Serialize message field [PI3]
    bufferOffset = _serializer.bool(obj.PI3, buffer, bufferOffset);
    // Serialize message field [PI4]
    bufferOffset = _serializer.bool(obj.PI4, buffer, bufferOffset);
    // Serialize message field [PI5]
    bufferOffset = _serializer.bool(obj.PI5, buffer, bufferOffset);
    // Serialize message field [PI6]
    bufferOffset = _serializer.bool(obj.PI6, buffer, bufferOffset);
    // Serialize message field [D5]
    bufferOffset = _serializer.bool(obj.D5, buffer, bufferOffset);
    // Serialize message field [D6]
    bufferOffset = _serializer.bool(obj.D6, buffer, bufferOffset);
    // Serialize message field [D7]
    bufferOffset = _serializer.bool(obj.D7, buffer, bufferOffset);
    // Serialize message field [D8]
    bufferOffset = _serializer.bool(obj.D8, buffer, bufferOffset);
    // Serialize message field [V5]
    bufferOffset = _serializer.bool(obj.V5, buffer, bufferOffset);
    // Serialize message field [V6]
    bufferOffset = _serializer.bool(obj.V6, buffer, bufferOffset);
    // Serialize message field [V7]
    bufferOffset = _serializer.bool(obj.V7, buffer, bufferOffset);
    // Serialize message field [V8]
    bufferOffset = _serializer.bool(obj.V8, buffer, bufferOffset);
    // Serialize message field [PS8]
    bufferOffset = _serializer.bool(obj.PS8, buffer, bufferOffset);
    // Serialize message field [PS9]
    bufferOffset = _serializer.bool(obj.PS9, buffer, bufferOffset);
    // Serialize message field [PS10]
    bufferOffset = _serializer.bool(obj.PS10, buffer, bufferOffset);
    // Serialize message field [PS11]
    bufferOffset = _serializer.bool(obj.PS11, buffer, bufferOffset);
    // Serialize message field [PS12]
    bufferOffset = _serializer.bool(obj.PS12, buffer, bufferOffset);
    // Serialize message field [PS13]
    bufferOffset = _serializer.bool(obj.PS13, buffer, bufferOffset);
    // Serialize message field [PS14]
    bufferOffset = _serializer.bool(obj.PS14, buffer, bufferOffset);
    // Serialize message field [PS15]
    bufferOffset = _serializer.bool(obj.PS15, buffer, bufferOffset);
    // Serialize message field [PS16]
    bufferOffset = _serializer.bool(obj.PS16, buffer, bufferOffset);
    // Serialize message field [PS17]
    bufferOffset = _serializer.bool(obj.PS17, buffer, bufferOffset);
    // Serialize message field [D5D]
    bufferOffset = _serializer.bool(obj.D5D, buffer, bufferOffset);
    // Serialize message field [D6D]
    bufferOffset = _serializer.bool(obj.D6D, buffer, bufferOffset);
    // Serialize message field [D7D]
    bufferOffset = _serializer.bool(obj.D7D, buffer, bufferOffset);
    // Serialize message field [D8D]
    bufferOffset = _serializer.bool(obj.D8D, buffer, bufferOffset);
    // Serialize message field [D5G]
    bufferOffset = _serializer.bool(obj.D5G, buffer, bufferOffset);
    // Serialize message field [D6G]
    bufferOffset = _serializer.bool(obj.D6G, buffer, bufferOffset);
    // Serialize message field [D7G]
    bufferOffset = _serializer.bool(obj.D7G, buffer, bufferOffset);
    // Serialize message field [D8G]
    bufferOffset = _serializer.bool(obj.D8G, buffer, bufferOffset);
    // Serialize message field [CPI3]
    bufferOffset = _serializer.bool(obj.CPI3, buffer, bufferOffset);
    // Serialize message field [CPI4]
    bufferOffset = _serializer.bool(obj.CPI4, buffer, bufferOffset);
    // Serialize message field [CPI5]
    bufferOffset = _serializer.bool(obj.CPI5, buffer, bufferOffset);
    // Serialize message field [CPI6]
    bufferOffset = _serializer.bool(obj.CPI6, buffer, bufferOffset);
    // Serialize message field [CP4]
    bufferOffset = _serializer.bool(obj.CP4, buffer, bufferOffset);
    // Serialize message field [CP5]
    bufferOffset = _serializer.bool(obj.CP5, buffer, bufferOffset);
    // Serialize message field [CP6]
    bufferOffset = _serializer.bool(obj.CP6, buffer, bufferOffset);
    // Serialize message field [CP7]
    bufferOffset = _serializer.bool(obj.CP7, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Retour_cellule_104Response
    let len;
    let data = new Retour_cellule_104Response(null);
    // Deserialize message field [ST8]
    data.ST8 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST9]
    data.ST9 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST10]
    data.ST10 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST11]
    data.ST11 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST12]
    data.ST12 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST13]
    data.ST13 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST14]
    data.ST14 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST15]
    data.ST15 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST16]
    data.ST16 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ST17]
    data.ST17 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R5D]
    data.R5D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R6D]
    data.R6D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R7D]
    data.R7D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R8D]
    data.R8D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R5G]
    data.R5G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R6G]
    data.R6G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R7G]
    data.R7G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [R8G]
    data.R8G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PI3]
    data.PI3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PI4]
    data.PI4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PI5]
    data.PI5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PI6]
    data.PI6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D5]
    data.D5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D6]
    data.D6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D7]
    data.D7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D8]
    data.D8 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V5]
    data.V5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V6]
    data.V6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V7]
    data.V7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [V8]
    data.V8 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS8]
    data.PS8 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS9]
    data.PS9 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS10]
    data.PS10 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS11]
    data.PS11 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS12]
    data.PS12 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS13]
    data.PS13 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS14]
    data.PS14 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS15]
    data.PS15 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS16]
    data.PS16 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [PS17]
    data.PS17 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D5D]
    data.D5D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D6D]
    data.D6D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D7D]
    data.D7D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D8D]
    data.D8D = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D5G]
    data.D5G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D6G]
    data.D6G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D7G]
    data.D7G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [D8G]
    data.D8G = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CPI3]
    data.CPI3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CPI4]
    data.CPI4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CPI5]
    data.CPI5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CPI6]
    data.CPI6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CP4]
    data.CP4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CP5]
    data.CP5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CP6]
    data.CP6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CP7]
    data.CP7 = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'schneider_104/Retour_cellule_104Response';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2f36819397689b3da7a9f133a5a1cfa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ST8
    bool ST9
    bool ST10
    bool ST11
    bool ST12
    bool ST13
    bool ST14
    bool ST15
    bool ST16
    bool ST17
    bool R5D
    bool R6D
    bool R7D
    bool R8D
    bool R5G
    bool R6G
    bool R7G
    bool R8G
    bool PI3
    bool PI4
    bool PI5
    bool PI6
    bool D5
    bool D6
    bool D7
    bool D8
    bool V5
    bool V6
    bool V7
    bool V8
    bool PS8
    bool PS9
    bool PS10
    bool PS11
    bool PS12
    bool PS13
    bool PS14
    bool PS15
    bool PS16
    bool PS17
    bool D5D
    bool D6D
    bool D7D
    bool D8D
    bool D5G
    bool D6G
    bool D7G
    bool D8G
    bool CPI3
    bool CPI4
    bool CPI5
    bool CPI6
    bool CP4
    bool CP5
    bool CP6
    bool CP7
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Retour_cellule_104Response(null);
    if (msg.ST8 !== undefined) {
      resolved.ST8 = msg.ST8;
    }
    else {
      resolved.ST8 = false
    }

    if (msg.ST9 !== undefined) {
      resolved.ST9 = msg.ST9;
    }
    else {
      resolved.ST9 = false
    }

    if (msg.ST10 !== undefined) {
      resolved.ST10 = msg.ST10;
    }
    else {
      resolved.ST10 = false
    }

    if (msg.ST11 !== undefined) {
      resolved.ST11 = msg.ST11;
    }
    else {
      resolved.ST11 = false
    }

    if (msg.ST12 !== undefined) {
      resolved.ST12 = msg.ST12;
    }
    else {
      resolved.ST12 = false
    }

    if (msg.ST13 !== undefined) {
      resolved.ST13 = msg.ST13;
    }
    else {
      resolved.ST13 = false
    }

    if (msg.ST14 !== undefined) {
      resolved.ST14 = msg.ST14;
    }
    else {
      resolved.ST14 = false
    }

    if (msg.ST15 !== undefined) {
      resolved.ST15 = msg.ST15;
    }
    else {
      resolved.ST15 = false
    }

    if (msg.ST16 !== undefined) {
      resolved.ST16 = msg.ST16;
    }
    else {
      resolved.ST16 = false
    }

    if (msg.ST17 !== undefined) {
      resolved.ST17 = msg.ST17;
    }
    else {
      resolved.ST17 = false
    }

    if (msg.R5D !== undefined) {
      resolved.R5D = msg.R5D;
    }
    else {
      resolved.R5D = false
    }

    if (msg.R6D !== undefined) {
      resolved.R6D = msg.R6D;
    }
    else {
      resolved.R6D = false
    }

    if (msg.R7D !== undefined) {
      resolved.R7D = msg.R7D;
    }
    else {
      resolved.R7D = false
    }

    if (msg.R8D !== undefined) {
      resolved.R8D = msg.R8D;
    }
    else {
      resolved.R8D = false
    }

    if (msg.R5G !== undefined) {
      resolved.R5G = msg.R5G;
    }
    else {
      resolved.R5G = false
    }

    if (msg.R6G !== undefined) {
      resolved.R6G = msg.R6G;
    }
    else {
      resolved.R6G = false
    }

    if (msg.R7G !== undefined) {
      resolved.R7G = msg.R7G;
    }
    else {
      resolved.R7G = false
    }

    if (msg.R8G !== undefined) {
      resolved.R8G = msg.R8G;
    }
    else {
      resolved.R8G = false
    }

    if (msg.PI3 !== undefined) {
      resolved.PI3 = msg.PI3;
    }
    else {
      resolved.PI3 = false
    }

    if (msg.PI4 !== undefined) {
      resolved.PI4 = msg.PI4;
    }
    else {
      resolved.PI4 = false
    }

    if (msg.PI5 !== undefined) {
      resolved.PI5 = msg.PI5;
    }
    else {
      resolved.PI5 = false
    }

    if (msg.PI6 !== undefined) {
      resolved.PI6 = msg.PI6;
    }
    else {
      resolved.PI6 = false
    }

    if (msg.D5 !== undefined) {
      resolved.D5 = msg.D5;
    }
    else {
      resolved.D5 = false
    }

    if (msg.D6 !== undefined) {
      resolved.D6 = msg.D6;
    }
    else {
      resolved.D6 = false
    }

    if (msg.D7 !== undefined) {
      resolved.D7 = msg.D7;
    }
    else {
      resolved.D7 = false
    }

    if (msg.D8 !== undefined) {
      resolved.D8 = msg.D8;
    }
    else {
      resolved.D8 = false
    }

    if (msg.V5 !== undefined) {
      resolved.V5 = msg.V5;
    }
    else {
      resolved.V5 = false
    }

    if (msg.V6 !== undefined) {
      resolved.V6 = msg.V6;
    }
    else {
      resolved.V6 = false
    }

    if (msg.V7 !== undefined) {
      resolved.V7 = msg.V7;
    }
    else {
      resolved.V7 = false
    }

    if (msg.V8 !== undefined) {
      resolved.V8 = msg.V8;
    }
    else {
      resolved.V8 = false
    }

    if (msg.PS8 !== undefined) {
      resolved.PS8 = msg.PS8;
    }
    else {
      resolved.PS8 = false
    }

    if (msg.PS9 !== undefined) {
      resolved.PS9 = msg.PS9;
    }
    else {
      resolved.PS9 = false
    }

    if (msg.PS10 !== undefined) {
      resolved.PS10 = msg.PS10;
    }
    else {
      resolved.PS10 = false
    }

    if (msg.PS11 !== undefined) {
      resolved.PS11 = msg.PS11;
    }
    else {
      resolved.PS11 = false
    }

    if (msg.PS12 !== undefined) {
      resolved.PS12 = msg.PS12;
    }
    else {
      resolved.PS12 = false
    }

    if (msg.PS13 !== undefined) {
      resolved.PS13 = msg.PS13;
    }
    else {
      resolved.PS13 = false
    }

    if (msg.PS14 !== undefined) {
      resolved.PS14 = msg.PS14;
    }
    else {
      resolved.PS14 = false
    }

    if (msg.PS15 !== undefined) {
      resolved.PS15 = msg.PS15;
    }
    else {
      resolved.PS15 = false
    }

    if (msg.PS16 !== undefined) {
      resolved.PS16 = msg.PS16;
    }
    else {
      resolved.PS16 = false
    }

    if (msg.PS17 !== undefined) {
      resolved.PS17 = msg.PS17;
    }
    else {
      resolved.PS17 = false
    }

    if (msg.D5D !== undefined) {
      resolved.D5D = msg.D5D;
    }
    else {
      resolved.D5D = false
    }

    if (msg.D6D !== undefined) {
      resolved.D6D = msg.D6D;
    }
    else {
      resolved.D6D = false
    }

    if (msg.D7D !== undefined) {
      resolved.D7D = msg.D7D;
    }
    else {
      resolved.D7D = false
    }

    if (msg.D8D !== undefined) {
      resolved.D8D = msg.D8D;
    }
    else {
      resolved.D8D = false
    }

    if (msg.D5G !== undefined) {
      resolved.D5G = msg.D5G;
    }
    else {
      resolved.D5G = false
    }

    if (msg.D6G !== undefined) {
      resolved.D6G = msg.D6G;
    }
    else {
      resolved.D6G = false
    }

    if (msg.D7G !== undefined) {
      resolved.D7G = msg.D7G;
    }
    else {
      resolved.D7G = false
    }

    if (msg.D8G !== undefined) {
      resolved.D8G = msg.D8G;
    }
    else {
      resolved.D8G = false
    }

    if (msg.CPI3 !== undefined) {
      resolved.CPI3 = msg.CPI3;
    }
    else {
      resolved.CPI3 = false
    }

    if (msg.CPI4 !== undefined) {
      resolved.CPI4 = msg.CPI4;
    }
    else {
      resolved.CPI4 = false
    }

    if (msg.CPI5 !== undefined) {
      resolved.CPI5 = msg.CPI5;
    }
    else {
      resolved.CPI5 = false
    }

    if (msg.CPI6 !== undefined) {
      resolved.CPI6 = msg.CPI6;
    }
    else {
      resolved.CPI6 = false
    }

    if (msg.CP4 !== undefined) {
      resolved.CP4 = msg.CP4;
    }
    else {
      resolved.CP4 = false
    }

    if (msg.CP5 !== undefined) {
      resolved.CP5 = msg.CP5;
    }
    else {
      resolved.CP5 = false
    }

    if (msg.CP6 !== undefined) {
      resolved.CP6 = msg.CP6;
    }
    else {
      resolved.CP6 = false
    }

    if (msg.CP7 !== undefined) {
      resolved.CP7 = msg.CP7;
    }
    else {
      resolved.CP7 = false
    }

    return resolved;
    }
};

module.exports = {
  Request: Retour_cellule_104Request,
  Response: Retour_cellule_104Response,
  md5sum() { return 'da2dff768924005c2c1434c86e6e1a14'; },
  datatype() { return 'schneider_104/Retour_cellule_104'; }
};
