#ifndef CELLULE_TP_H
#define CELLULE_TP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <schneider_103/Retour_cellule_103.h>
#include <schneider_103/Msg_SensorState.h>
#include "commande_locale/Msg_StopControl.h"
#include <commande_locale/Msg_ChoixMode.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>

using namespace std;

class Cellule_tp{

private:
	ros::Publisher pub_103;
	ros::Publisher cap_103;
	std_msgs::String msg;
	ros::ServiceClient client;
	schneider_103::Retour_cellule_103 srv;
	string data;
	ros::Subscriber cmd_aigGauche_cell_103;
	ros::Subscriber cmd_aigDroite_cell_103;
	ros::Subscriber cmd_PS_103;
	schneider_103::Msg_SensorState SensorState;
	ros::Subscriber choixMode;
	int mode;
	enum Actionneur_{ST6 = 0,
					 ST7 = 1,
					 ST18 = 2,
					 ST19 = 3,
					 R3D = 4,
					 R4D = 5,
					 R9D = 6,
					 R10D = 7,
					 R3G = 8,
					 R4G = 9,
					 R9G = 10,
					 R10G = 11,
					 D3 = 12,
					 D4 = 13,
					 D9 = 14,
					 D10 = 15,
					 V3 = 16,
					 V4 = 17,
					 V9 = 18,
					 V10 = 19};
	enum Aiguillage_{A3 = 3,
					 A4 = 4,
					 A9 = 9,
					 A10 = 10};
	
public:
	Cellule_tp(ros::NodeHandle noeud);
	~Cellule_tp();
	void read();
	void write(vector<vector<int>> consigne);
	void AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs);
	void AigDroiteCallback(const std_msgs::Int32::ConstPtr& msg_aigs);
	void CmdPSCallback(const commande_locale::Msg_StopControl actionneurs_simulation_Stop);
	void TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1);

	
};
#endif
