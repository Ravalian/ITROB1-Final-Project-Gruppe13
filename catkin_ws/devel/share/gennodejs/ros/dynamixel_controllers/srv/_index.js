
"use strict";

let StopController = require('./StopController.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let StartController = require('./StartController.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let RestartController = require('./RestartController.js')
let SetSpeed = require('./SetSpeed.js')

module.exports = {
  StopController: StopController,
  SetComplianceMargin: SetComplianceMargin,
  StartController: StartController,
  SetCompliancePunch: SetCompliancePunch,
  TorqueEnable: TorqueEnable,
  SetTorqueLimit: SetTorqueLimit,
  SetComplianceSlope: SetComplianceSlope,
  RestartController: RestartController,
  SetSpeed: SetSpeed,
};
