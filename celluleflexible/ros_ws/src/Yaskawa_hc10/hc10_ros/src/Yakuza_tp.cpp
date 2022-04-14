#include "Yakuza_tp.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;


Yakuza_tp::Yakuza_tp(ros::NodeHandle noeud):mode{0}, yaskaType{0}
{	
	
	choixMode = noeud.subscribe("/commande_locale/ChoixMode", 10,&Yakuza_tp::TypeMode,this);
	robot=noeud.subscribe("/commande/Simulation/DeplacerPiece",10,&Yakuza_tp::RobCallabck,this);
	pub_yaku = noeud.advertise<std_msgs::Int32>("/control_yakuza", 1);
	
}

Yakuza_tp::~Yakuza_tp()
{
}

void Yakuza_tp::TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1)
{
	mode = msg1->mode;
	yaskaType = msg1->yaska;

}

void Yakuza_tp::RobCallabck(const commande_locale::DeplacerPieceMsg msg)
{
	if( (mode==0 && yaskaType==1) || (mode == 1 && yaskaType == 2) ){
		if (msg.num_robot==4)
		{
			if(msg.positionA==2)
			{
				DeplacementRobot.data=1;
			}
			else if(msg.positionA==3)
			{
				
				DeplacementRobot.data=3;
			}
			else if (msg.positionB==2)
			{
				DeplacementRobot.data=2;
			}
			else if (msg.positionB==3)
			{
				DeplacementRobot.data=4;
			}
			pub_yaku.publish(DeplacementRobot);
		}
	

	}


}









