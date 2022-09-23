#include "robotic_platform.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <thread> 
#include <chrono>
#include <iostream>
#include <math.h>
using namespace std;

// ERGOTS PI A intégrer dans le déplacement piece et se désactive avec la commande Ouvrir_PS_GO()
// Pour pouvoir enclencher l'ergot, il faut : que la navette soit en face du PS correspondant, que le PS correspondant soit a stop'

robotic_platform::robotic_platform(ros::NodeHandle noeud):yaska4Type_{Robot_::Coppelia},yaska1Type_{Robot_::Coppelia},kukaType_{Robot_::Coppelia},staubliType_{Robot_::Coppelia}
{	
	choixMode_ = noeud.subscribe("/commande_locale/ChoixMode", 10,&robotic_platform::TypeMode,this);
	robot_ = noeud.subscribe("/commande/Simulation/DeplacerPiece",10,&robotic_platform::RobCallabck,this);
	pub_robot_yaska4 = noeud.advertise<std_msgs::Int32>("/control_robot_yaska4", 1);
	pub_robot_yaska1 = noeud.advertise<std_msgs::Int32>("/control_robot_yaska1", 1);
	pub_robot_kuka = noeud.advertise<std_msgs::Int32>("/control_robot_kuka", 1);
	pub_robot_staubli = noeud.advertise<std_msgs::Int32>("/control_robot_staubli", 1);

}

robotic_platform::~robotic_platform()
{
}

void robotic_platform::TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1)
{
	staubliType_ = msg1->staubli;
	kukaType_ = msg1->kuka;
	yaska1Type_ = msg1->yaska1;
	yaska4Type_ = msg1->yaska4;
}

void robotic_platform::RobCallabck(const commande_locale::DeplacerPieceMsg msg)
{
	switch(msg.positionA){
		// Déplacement du poste 1 vers navette
		case 1:
			if(msg.positionB==2){
				DeplacementRobot_.data=Group_::DP1N2;
			}
			else if(msg.positionB==3){
				DeplacementRobot_.data=Group_::DP1N3;
			}
			else{
				DeplacementRobot_.data=-1;
				ROS_INFO("Wrong Deplacement requested");
			}
			break;
		// Déplacement de navette 2 vers poste
		case 2:
			if(msg.positionB==1){
				DeplacementRobot_.data=Group_::DN2P1;
			}
			else if(msg.positionB==4){
				DeplacementRobot_.data=Group_::DN2P4;
			}
			else{
				DeplacementRobot_.data=-1;
				ROS_INFO("Wrong Deplacement requested");
			}
			break;
		// Déplacement de navette 3 vers poste
		case 3:
			if(msg.positionB==1){
				DeplacementRobot_.data=Group_::DN3P1;
			}
			else if(msg.positionB==4){
				DeplacementRobot_.data=Group_::DN3P4;
			}
			else{
				DeplacementRobot_.data=-1;
				ROS_INFO("Wrong Deplacement requested");
			}
			break;
		// Déplacement de poste 4 vers navette
		case 4:
			if(msg.positionB==2){
				DeplacementRobot_.data=Group_::DP4N2;
			}
			else if(msg.positionB==3){
				DeplacementRobot_.data=Group_::DP4N3;
			}
			else{
				DeplacementRobot_.data=-1;
				ROS_INFO("Wrong Deplacement requested");
			}
			break;
		default:
			DeplacementRobot_.data=-1;
			ROS_INFO("Wrong Deplacement requested");
			break;
	}
	if(DeplacementRobot_.data != -1){
		switch(msg.num_robot){
			case 1:
				//robot position basse
				if(yaska1Type_ != Robot_::Coppelia){
					pub_robot_yaska1.publish(DeplacementRobot_);
				}
				else ROS_INFO("Wrong mode of robot yaska1, change parameter");
				break;
			case 2:
				//robot position haute
				if(kukaType_ != Robot_::Coppelia){
					pub_robot_kuka.publish(DeplacementRobot_);
				}
				else ROS_INFO("Wrong mode of robot kuka, change parameter");
				break;
			case 3:
				//robot position haute
				if(staubliType_ != Robot_::Coppelia){
					pub_robot_staubli.publish(DeplacementRobot_);
				}
				else ROS_INFO("Wrong mode of robot staubli, change parameter");
				break;
			case 4:
				//robot position basse
				if(yaska4Type_ != Robot_::Coppelia){
					pub_robot_yaska4.publish(DeplacementRobot_);
				}
				else ROS_INFO("Wrong mode of robot yaska4, change parameter");
				break;
			default:
				ROS_INFO("Wrong robot number");
				break;
		}
	}
}









