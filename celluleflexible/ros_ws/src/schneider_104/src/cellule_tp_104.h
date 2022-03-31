#ifndef CELLULE_TP_H
#define CELLULE_TP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <schneider_104/Retour_cellule_104.h>
#include <schneider_104/Msg_SensorState.h>
#include "commande_locale/Msg_StopControl.h"
#include <commande_locale/Msg_ChoixMode.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>
using namespace std;



class Cellule_tp
{

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
	schneider_104::Msg_SensorState SensorState;
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
	bool ST8;
	bool ST9;
	bool ST10;
	bool ST11;
	bool ST12;
	bool ST13;
	bool ST14;
	bool ST15;
	bool ST16;
	bool ST17;
	bool R5D;
	bool R6D;
	bool R7D;
	bool R8D;
	bool R5G;
	bool R6G;
	bool R7G;
	bool R8G;
	bool PI3;
	bool PI4;
	bool PI5;
	bool PI6;
	bool D5;
	bool D6;
	bool D7;
	bool D8;
	bool V5;
	bool V6;
	bool V7;
	bool V8;
	bool PS8;
	bool PS9;
	bool PS10;
	bool PS11;
	bool PS12;
	bool PS13;
	bool PS14;
	bool PS15;
	bool PS16;
	bool PS17;
	bool D5D;
	bool D6D;
	bool D7D;
	bool D8D;
	bool D5G;
	bool D6G;
	bool D7G;
	bool D8G;
	bool CPI3;
	bool CPI4;
	bool CPI5;
	bool CPI6;
	bool CP4;
	bool CP5;
	bool CP6;
	bool CP7;
	
	
};

#endif
