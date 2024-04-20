// Auto-generated. Do not edit!

// (in-package rft_sensor_serial.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class rft_operationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.opType = null;
      this.param1 = null;
      this.param2 = null;
      this.param3 = null;
    }
    else {
      if (initObj.hasOwnProperty('opType')) {
        this.opType = initObj.opType
      }
      else {
        this.opType = 0;
      }
      if (initObj.hasOwnProperty('param1')) {
        this.param1 = initObj.param1
      }
      else {
        this.param1 = 0;
      }
      if (initObj.hasOwnProperty('param2')) {
        this.param2 = initObj.param2
      }
      else {
        this.param2 = 0;
      }
      if (initObj.hasOwnProperty('param3')) {
        this.param3 = initObj.param3
      }
      else {
        this.param3 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rft_operationRequest
    // Serialize message field [opType]
    bufferOffset = _serializer.uint8(obj.opType, buffer, bufferOffset);
    // Serialize message field [param1]
    bufferOffset = _serializer.uint8(obj.param1, buffer, bufferOffset);
    // Serialize message field [param2]
    bufferOffset = _serializer.uint8(obj.param2, buffer, bufferOffset);
    // Serialize message field [param3]
    bufferOffset = _serializer.uint8(obj.param3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rft_operationRequest
    let len;
    let data = new rft_operationRequest(null);
    // Deserialize message field [opType]
    data.opType = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [param1]
    data.param1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [param2]
    data.param2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [param3]
    data.param3 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rft_sensor_serial/rft_operationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76ad4c31d463fda0980c1dc1fd97aa8b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 opType
    uint8 param1
    uint8 param2
    uint8 param3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rft_operationRequest(null);
    if (msg.opType !== undefined) {
      resolved.opType = msg.opType;
    }
    else {
      resolved.opType = 0
    }

    if (msg.param1 !== undefined) {
      resolved.param1 = msg.param1;
    }
    else {
      resolved.param1 = 0
    }

    if (msg.param2 !== undefined) {
      resolved.param2 = msg.param2;
    }
    else {
      resolved.param2 = 0
    }

    if (msg.param3 !== undefined) {
      resolved.param3 = msg.param3;
    }
    else {
      resolved.param3 = 0
    }

    return resolved;
    }
};

class rft_operationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rft_operationResponse
    // Serialize message field [result]
    bufferOffset = _serializer.uint8(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rft_operationResponse
    let len;
    let data = new rft_operationResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rft_sensor_serial/rft_operationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '25458147911545c320c4c0a299eff763';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rft_operationResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: rft_operationRequest,
  Response: rft_operationResponse,
  md5sum() { return '5b818d3a3d5cb051d3d33d376492250b'; },
  datatype() { return 'rft_sensor_serial/rft_operation'; }
};
