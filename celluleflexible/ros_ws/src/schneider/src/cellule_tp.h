#ifndef CELLULE_TP_H
#define CELLULE_TP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <schneider/Retour_cellule.h>
#include <schneider/Msg_SensorState.h>
#include "commande_locale/Msg_StopControl.h"
#include <commande_locale/Msg_ChoixMode.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>
using namespace std;



class Cellule_tp
{

private:
	ros::Publisher pub;
	ros::Publisher cap;
	ros::Subscriber choixMode;
	std_msgs::String msg;
	ros::ServiceClient client;
	schneider::Retour_cellule srv;
	string data;
	ros::Subscriber cmd_aigGauche_cell;
	ros::Subscriber cmd_aigDroite_cell;
	ros::Subscriber cmd_PS;
	schneider::Msg_SensorState SensorState;
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
	bool ST1;
	bool ST2;
	bool ST3;
	bool ST4;
	bool ST5;
	bool ST20;
	bool ST21;
	bool ST22;
	bool ST23;
	bool ST24;
	bool R1D;
	bool R2D;
	bool R11D;
	bool R12D;
	bool R1G;
	bool R2G;
	bool R11G;
	bool R12G;
	bool PI1;
	bool PI2;
	bool PI7;
	bool PI8;
	bool D1;
	bool D2;
	bool D11;
	bool D12;
	bool V1;
	bool V2;
	bool V11;
	bool V12;
	bool PS1;
	bool PS2;
	bool PS3;
	bool PS4;
	bool PS5;
	bool PS20;
	bool PS21;
	bool PS22;
	bool PS23;
	bool PS24;
	bool D1D;
	bool D2D;
	bool D11D;
	bool D12D;
	bool D1G;
	bool D2G;
	bool D11G;
	bool D12G;
	bool CPI1;
	bool CPI2;
	bool CPI7;
	bool CPI8;
	bool CP1;
	bool CP2;
	bool CP9;
	bool CP10;
	
	
};

#endif
