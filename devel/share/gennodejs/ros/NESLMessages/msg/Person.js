// Auto-generated. Do not edit!

// (in-package NESLMessages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NeslCoord = require('./NeslCoord.js');

//-----------------------------------------------------------

class Person {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.colorArr = null;
      this.personID = null;
      this.personCoord = null;
      this.accountedFor = null;
      this.talking = null;
      this.bbx = null;
      this.bby = null;
      this.bbz = null;
    }
    else {
      if (initObj.hasOwnProperty('colorArr')) {
        this.colorArr = initObj.colorArr
      }
      else {
        this.colorArr = [];
      }
      if (initObj.hasOwnProperty('personID')) {
        this.personID = initObj.personID
      }
      else {
        this.personID = 0;
      }
      if (initObj.hasOwnProperty('personCoord')) {
        this.personCoord = initObj.personCoord
      }
      else {
        this.personCoord = new NeslCoord();
      }
      if (initObj.hasOwnProperty('accountedFor')) {
        this.accountedFor = initObj.accountedFor
      }
      else {
        this.accountedFor = false;
      }
      if (initObj.hasOwnProperty('talking')) {
        this.talking = initObj.talking
      }
      else {
        this.talking = false;
      }
      if (initObj.hasOwnProperty('bbx')) {
        this.bbx = initObj.bbx
      }
      else {
        this.bbx = 0.0;
      }
      if (initObj.hasOwnProperty('bby')) {
        this.bby = initObj.bby
      }
      else {
        this.bby = 0.0;
      }
      if (initObj.hasOwnProperty('bbz')) {
        this.bbz = initObj.bbz
      }
      else {
        this.bbz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Person
    // Serialize message field [colorArr]
    bufferOffset = _arraySerializer.float64(obj.colorArr, buffer, bufferOffset, null);
    // Serialize message field [personID]
    bufferOffset = _serializer.int32(obj.personID, buffer, bufferOffset);
    // Serialize message field [personCoord]
    bufferOffset = NeslCoord.serialize(obj.personCoord, buffer, bufferOffset);
    // Serialize message field [accountedFor]
    bufferOffset = _serializer.bool(obj.accountedFor, buffer, bufferOffset);
    // Serialize message field [talking]
    bufferOffset = _serializer.bool(obj.talking, buffer, bufferOffset);
    // Serialize message field [bbx]
    bufferOffset = _serializer.float32(obj.bbx, buffer, bufferOffset);
    // Serialize message field [bby]
    bufferOffset = _serializer.float32(obj.bby, buffer, bufferOffset);
    // Serialize message field [bbz]
    bufferOffset = _serializer.float32(obj.bbz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Person
    let len;
    let data = new Person(null);
    // Deserialize message field [colorArr]
    data.colorArr = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [personID]
    data.personID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [personCoord]
    data.personCoord = NeslCoord.deserialize(buffer, bufferOffset);
    // Deserialize message field [accountedFor]
    data.accountedFor = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [talking]
    data.talking = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [bbx]
    data.bbx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bby]
    data.bby = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bbz]
    data.bbz = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.colorArr.length;
    return length + 46;
  }

  static datatype() {
    // Returns string type for a message object
    return 'NESLMessages/Person';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '07061084615db4a1b1a7a2a5b8b45019';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] colorArr
    int32 personID
    NeslCoord personCoord
    bool accountedFor
    bool talking
    float32 bbx
    float32 bby
    float32 bbz
    
    ================================================================================
    MSG: NESLMessages/NeslCoord
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Person(null);
    if (msg.colorArr !== undefined) {
      resolved.colorArr = msg.colorArr;
    }
    else {
      resolved.colorArr = []
    }

    if (msg.personID !== undefined) {
      resolved.personID = msg.personID;
    }
    else {
      resolved.personID = 0
    }

    if (msg.personCoord !== undefined) {
      resolved.personCoord = NeslCoord.Resolve(msg.personCoord)
    }
    else {
      resolved.personCoord = new NeslCoord()
    }

    if (msg.accountedFor !== undefined) {
      resolved.accountedFor = msg.accountedFor;
    }
    else {
      resolved.accountedFor = false
    }

    if (msg.talking !== undefined) {
      resolved.talking = msg.talking;
    }
    else {
      resolved.talking = false
    }

    if (msg.bbx !== undefined) {
      resolved.bbx = msg.bbx;
    }
    else {
      resolved.bbx = 0.0
    }

    if (msg.bby !== undefined) {
      resolved.bby = msg.bby;
    }
    else {
      resolved.bby = 0.0
    }

    if (msg.bbz !== undefined) {
      resolved.bbz = msg.bbz;
    }
    else {
      resolved.bbz = 0.0
    }

    return resolved;
    }
};

module.exports = Person;
