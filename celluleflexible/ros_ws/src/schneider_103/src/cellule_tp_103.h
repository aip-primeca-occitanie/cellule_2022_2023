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



class Cellule_tp
{

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
	
public:
	Cellule_tp(ros::NodeHandle noeud);
	~Cellule_tp();
	void read();
	void write(vector<vector<int>> consigne);
	void AigGaucheCallback(const std_msgs::Int32::ConstPtr& msg_aigs);
	void AigDroiteCallback(const std_msgs::Int32::ConstPtr& msg_aigs);
	void CmdPSCallback(const commande_locale::Msg_StopControl actionneurs_simulation_Stop);
	void TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1);
	bool ST6;
	bool ST7;
	bool ST18;
	bool ST19;
	bool R3D;
	bool R4D;
	bool R9D;
	bool R10D;
	bool R3G;
	bool R4G;
	bool R9G;
	bool R10G;
	bool D3;
	bool D4;
	bool D9;
	bool D10;
	bool V3;
	bool V4;
	bool V9;
	bool V10;
	bool PS7;
	bool PS18;
	bool PS19;
	bool D3D;
	bool D4D;
	bool D9D;
	bool D10D;
	bool D3G;
	bool D4G;
	bool D9G;
	bool D10G;
	bool CP3;
	bool CP8;

	
};

#endif
