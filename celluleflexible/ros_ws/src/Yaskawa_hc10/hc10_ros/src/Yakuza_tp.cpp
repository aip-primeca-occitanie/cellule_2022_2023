#include "Yakuza_tp.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;


Yakuza_tp::Yakuza_tp(ros::NodeHandle noeud):yaskaType_{Yaska_::Coppelia}
{	
	
	choixMode_ = noeud.subscribe("/commande_locale/ChoixMode", 10,&Yakuza_tp::TypeMode,this);
	robot_ = noeud.subscribe("/commande/Simulation/DeplacerPiece",10,&Yakuza_tp::RobCallabck,this);
	pub_yaku_ = noeud.advertise<std_msgs::Int32>("/control_yakuza", 1);

}

Yakuza_tp::~Yakuza_tp()
{
}

void Yakuza_tp::TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1)
{
	yaskaType_ = msg1->yaska;
}

void Yakuza_tp::RobCallabck(const commande_locale::DeplacerPieceMsg msg)
{
	if( yaskaType_ != Yaska_::Coppelia ){
		if (msg.num_robot==4)
		{
			if(msg.positionA==2)
			{
				DeplacementRobot_.data=Group_::DN1P;
			}
			else if(msg.positionA==3)
			{
				
				DeplacementRobot_.data=Group_::DPN1;
			}
			else if (msg.positionB==2)
			{
				DeplacementRobot_.data=Group_::DN2P;
			}
			else if (msg.positionB==3)
			{
				DeplacementRobot_.data=Group_::DPN2;
			}
			pub_yaku_.publish(DeplacementRobot_);
		}
	

	}
	else{ std::cout << "Changez le mode Yaskawa dans les paramètres pour visualiser les trajectoires" << std::endl; };

}









