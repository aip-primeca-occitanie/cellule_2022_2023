
"use strict";

let Msg_SwitchControl = require('./Msg_SwitchControl.js');
let TacheFinieMsg = require('./TacheFinieMsg.js');
let Msg_Color = require('./Msg_Color.js');
let DeplacerPieceMsg = require('./DeplacerPieceMsg.js');
let Msg_AddProduct = require('./Msg_AddProduct.js');
let Capteurs = require('./Capteurs.js');
let Msg_Erreur = require('./Msg_Erreur.js');
let Msg_StopControl = require('./Msg_StopControl.js');
let Actionneurs = require('./Actionneurs.js');
let Msg_PinControl = require('./Msg_PinControl.js');
let Msg_ChoixMode = require('./Msg_ChoixMode.js');
let RobotJoints = require('./RobotJoints.js');

module.exports = {
  Msg_SwitchControl: Msg_SwitchControl,
  TacheFinieMsg: TacheFinieMsg,
  Msg_Color: Msg_Color,
  DeplacerPieceMsg: DeplacerPieceMsg,
  Msg_AddProduct: Msg_AddProduct,
  Capteurs: Capteurs,
  Msg_Erreur: Msg_Erreur,
  Msg_StopControl: Msg_StopControl,
  Actionneurs: Actionneurs,
  Msg_PinControl: Msg_PinControl,
  Msg_ChoixMode: Msg_ChoixMode,
  RobotJoints: RobotJoints,
};
