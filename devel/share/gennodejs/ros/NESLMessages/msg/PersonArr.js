// Auto-generated. Do not edit!

// (in-package NESLMessages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Person = require('./Person.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PersonArr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.personArr = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('personArr')) {
        this.personArr = initObj.personArr
      }
      else {
        this.personArr = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PersonArr
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [personArr]
    // Serialize the length for message field [personArr]
    bufferOffset = _serializer.uint32(obj.personArr.length, buffer, bufferOffset);
    obj.personArr.forEach((val) => {
      bufferOffset = Person.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PersonArr
    let len;
    let data = new PersonArr(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [personArr]
    // Deserialize array length for message field [personArr]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.personArr = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.personArr[i] = Person.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.personArr.forEach((val) => {
      length += Person.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'NESLMessages/PersonArr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6fc894af3e67e11ac542d4784fd5fbb1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Person[] personArr
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: NESLMessages/Person
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
    const resolved = new PersonArr(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.personArr !== undefined) {
      resolved.personArr = new Array(msg.personArr.length);
      for (let i = 0; i < resolved.personArr.length; ++i) {
        resolved.personArr[i] = Person.Resolve(msg.personArr[i]);
      }
    }
    else {
      resolved.personArr = []
    }

    return resolved;
    }
};

module.exports = PersonArr;
