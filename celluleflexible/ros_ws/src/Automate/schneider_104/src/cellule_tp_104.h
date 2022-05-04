#ifndef CELLULE_TP_H
#define CELLULE_TP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <schneider_104/Retour_cellule_104.h>
#include <schneider_104/Msg_SensorState.h>
#include "commande_locale/Msg_StopControl.h"
#include <commande_locale/Msg_ChoixMode.h>
#include <commande_locale/DeplacerPieceMsg.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>

using namespace std;

class Cellule_tp{

private:
	ros::Publisher pub_104;
	ros::Publisher cap_104;
	std_msgs::String msg;
	ros::ServiceClient client;
	schneider_104::Retour_cellule_104 srv;
	string data;
	ros::Subscriber cmd_aigGauche_cell_104;
	ros::Subscriber cmd_aigDroite_cell_104;
	ros::Subscriber cmd_PS_104;
	//ros::Subscriber robot;
	schneider_104::Msg_SensorState SensorState;
	ros::Subscriber choixMode;
	int mode;
	//int isKukaPhysical;
	enum Actionneur_{ST8 = 0,
					 ST9 = 1,
					 ST10 = 2,
					 ST11 = 3,
					 ST12 = 4,
					 ST13 = 5,
					 ST14 = 6,
					 ST15 = 7,
					 ST16 = 8,
					 ST17 = 9,
					 R5D = 10,
					 R6D = 11,
					 R7D = 12,
					 R8D = 13,
					 R5G = 14,
					 R6G = 15,
					 R7G = 16,
					 R8G = 17,
					 PI3 = 18,
					 PI4 = 19,
					 PI5 = 20,
					 PI6 = 21,
					 D5 = 22,
					 D6 = 23,
					 D7 = 24,
					 D8 = 25,
					 V5 = 26,
					 V6 = 27,
					 V7 = 28,
					 V8 = 29,
					 OUTR10 = 34,
					 OUTR11 = 35};
	enum Aiguillage_{A5 = 5,
					 A6 = 6,
					 A7 = 7,
					 A8 = 8};
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
