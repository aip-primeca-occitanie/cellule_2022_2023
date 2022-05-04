#include "robotic_platform.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;


robotic_platform::robotic_platform(ros::NodeHandle noeud, std::string name):yaska4Type_{Robot_::Coppelia},yaska3Type_{Robot_::Coppelia},kukaType_{Robot_::Coppelia},staubliType_{Robot_::Coppelia}
{	
	name_ = name;
	choixMode_ = noeud.subscribe("/commande_locale/ChoixMode", 10,&robotic_platform::TypeMode,this);
	robot_ = noeud.subscribe("/commande/Simulation/DeplacerPiece",10,&robotic_platform::RobCallabck,this);
	pub_robot_ = noeud.advertise<std_msgs::Int32>("/control_robot", 1);

}

robotic_platform::~robotic_platform()
{
}

void robotic_platform::TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1)
{
	staubliType_ = msg1->staubli;
	kukaType_ = msg1->kuka;
	yaska3Type_ = msg1->yaska3;
	yaska4Type_ = msg1->yaska4;
}

void robotic_platform::RobCallabck(const commande_locale::DeplacerPieceMsg msg)
{
	switch(msg.num_robot){
		case 1:
			//robot position basse
			if(staubliType_ != Robot_::Coppelia && (name_.compare("staubli")==0)){
				// navette 2 vers poste 1
				if(msg.positionA==2){
					DeplacementRobot_.data=Group_::DN1P;
				}
				// navette 3 vers poste 4
				else if(msg.positionA==3){
					DeplacementRobot_.data=Group_::DPN1;
				}
				// poste 1 vers navette 2
				else if (msg.positionB==2){
					DeplacementRobot_.data=Group_::DN2P;
				}
				// poste 4 vers navette 3
				else if (msg.positionB==3){
					DeplacementRobot_.data=Group_::DPN2;
				}
				pub_robot_.publish(DeplacementRobot_);
			}
			else std::cout<<"Wrong mode or name of robot"<<std::endl;
			break;
		case 2:
			//robot position haute
			if(kukaType_ != Robot_::Coppelia && (name_.compare("kuka")==0)){
				// navette 2 vers poste 1
				if(msg.positionA==2){
					DeplacementRobot_.data=Group_::DN1P;
				}
				// navette 3 vers poste 4
				else if(msg.positionA==3){
					DeplacementRobot_.data=Group_::DPN1;
				}
				// poste 1 vers navette 2
				else if (msg.positionB==2){
					DeplacementRobot_.data=Group_::DN2P;
				}
				// poste 4 vers navette 3
				else if (msg.positionB==3){
					DeplacementRobot_.data=Group_::DPN2;
				}
				pub_robot_.publish(DeplacementRobot_);
			}
			else std::cout <<"Wrong mode or name of robot"<<std::endl;
			break;
		case 3:
			//robot position basse
			if(yaska3Type_ != Robot_::Coppelia && (name_.compare("yaska3")==0)){
				// navette 2 vers poste 1
				if(msg.positionA==2){
					DeplacementRobot_.data=Group_::DN1P;
				}
				// navette 3 vers poste 4
				else if(msg.positionA==3){
					DeplacementRobot_.data=Group_::DPN1;
				}
				// poste 1 vers navette 2
				else if (msg.positionB==2){
					DeplacementRobot_.data=Group_::DN2P;
				}
				// poste 4 vers navette 3
				else if (msg.positionB==3){
					DeplacementRobot_.data=Group_::DPN2;
				}
				pub_robot_.publish(DeplacementRobot_);
			}
			else std::cout << "Wrong mode or name of robot"<<std::endl;
			break;
		case 4:
			//robot position basse
			if(yaska4Type_ != Robot_::Coppelia && (name_.compare("yaska4")==0)){
				// navette 2 vers poste 1
				if(msg.positionA==2){
					DeplacementRobot_.data=Group_::DN1P;
				}
				// navette 3 vers poste 4
				else if(msg.positionA==3){
					DeplacementRobot_.data=Group_::DPN1;
				}
				// poste 1 vers navette 2
				else if (msg.positionB==2){
					DeplacementRobot_.data=Group_::DN2P;
				}
				// poste 4 vers navette 3
				else if (msg.positionB==3){
					DeplacementRobot_.data=Group_::DPN2;
				}
				pub_robot_.publish(DeplacementRobot_);
			}
			else std::cout << "Wrong mode or name of robot"<< std::endl;
			break;
		default:
			std::cout << "Wrong robot number" << std::endl;
			break;
	}
}









