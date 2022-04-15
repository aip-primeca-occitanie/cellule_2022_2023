#ifndef YAKUZA_TP_H
#define YAKUZA_TP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include "commande_locale/Msg_StopControl.h"
#include <commande_locale/Msg_ChoixMode.h>
#include <commande_locale/DeplacerPieceMsg.h>
#include <robots/FinDeplacerPiece_Msg.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>
using namespace std;



class Yakuza_tp
{

private:
	ros::Publisher pub_yaku_;
	ros::Subscriber choixMode_;
	ros::Subscriber robot_;
	std_msgs::Int32 DeplacementRobot_;
	int yaskaType_;
	enum Yaska_{Coppelia = 0, Rviz, Atelier};
	enum Group_{DN1P = 1, DN2P, DPN1, DPN2};
	
public:
	Yakuza_tp(ros::NodeHandle noeud);
	~Yakuza_tp();
	void RobCallabck(const commande_locale::DeplacerPieceMsg msg);
	void TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1);
	
};

#endif
