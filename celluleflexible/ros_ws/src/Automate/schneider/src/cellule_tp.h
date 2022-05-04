#ifndef CELLULE_TP_H
#define CELLULE_TP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <schneider/Retour_cellule.h>
#include <schneider/Msg_SensorState.h>
#include "commande_locale/Msg_StopControl.h"
#include <commande_locale/Msg_ChoixMode.h>
#include <commande_locale/DeplacerPieceMsg.h>
#include <robots/FinDeplacerPiece_Msg.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>
using namespace std;

class Cellule_tp{

private:
	ros::Publisher pub;
	//ros::Publisher pub_fintache;
	ros::Publisher cap;
	ros::Subscriber choixMode;
	std_msgs::String msg;
	ros::ServiceClient client;
	schneider::Retour_cellule srv;
	string data;
	ros::Subscriber cmd_aigGauche_cell;
	ros::Subscriber cmd_aigDroite_cell;
	ros::Subscriber cmd_PS;
	//ros::Subscriber robot;
	schneider::Msg_SensorState SensorState;
	//robots::FinDeplacerPiece_Msg SensorRobots;
	int mode;
	//int isKukaPhysical;
	enum Actionneur_{ST1 = 0,
					 ST2 = 1,
					 ST3 = 2,
					 ST4 = 3,
					 ST5 = 4,
					 ST20 = 5,
					 ST21 = 6,
					 ST22 = 7,
					 ST23 = 8,
					 ST24 = 9,
					 R1D = 10,
					 R2D = 11,
					 R11D = 12,
					 R12D = 13,
					 R1G = 14,
					 R2G = 15,
					 R11G = 16,
					 R12G = 17,
					 PI1 = 18,
					 PI2 = 19,
					 PI7 = 20,
					 PI8 = 21,
					 D1 = 22,
					 D2 = 23,
					 D11 = 24,
					 D12 = 25,
					 V1 = 26,
					 V2 = 27,
					 V11 = 28,
					 V12 = 29,
					 OUTR1 = 34,
					 OUTR2 = 35,
					 OUTR3 = 36,
					 OUTR4 = 37};
	enum Aiguillage_{A1 = 1,
					 A2 = 2,
					 A11 = 11,
					 A12 = 12};
public:
	Cellule_tp(ros::NodeHandle noeud);
	~Cellule_tp();
	void read();
	void write(vector<vector<int>> consigne);
	void AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs);
	void AigDroiteCallback(const std_msgs::Int32::ConstPtr& msg_aigs);
	void CmdPSCallback(const commande_locale::Msg_StopControl actionneurs_simulation_Stop);
	//void RobCallabck(const commande_locale::DeplacerPieceMsg msg);
	void TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1);

};
#endif
