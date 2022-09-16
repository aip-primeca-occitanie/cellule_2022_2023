#ifndef ROBOTIC_PLATFORM_H
#define ROBOTIC_PLATFORM_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <commande_locale/Msg_ChoixMode.h>
#include <commande_locale/DeplacerPieceMsg.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <std_msgs/Byte.h>
using namespace std;



class robotic_platform
{

private:
	ros::Publisher pub_robot_kuka;
	ros::Publisher pub_robot_staubli;
	ros::Publisher pub_robot_yaska1;
	ros::Publisher pub_robot_yaska4;
	ros::Subscriber choixMode_;
	ros::Subscriber robot_;
	std_msgs::Int32 DeplacementRobot_;
	int yaska4Type_;
	int yaska1Type_;
	int kukaType_;
	int staubliType_;
	enum Robot_{Coppelia = 0, Rviz, Atelier};
	enum Group_{DN1P = 1, DN2P, DPN1, DPN2};
	
public:
	robotic_platform(ros::NodeHandle noeud);
	~robotic_platform();
	void RobCallabck(const commande_locale::DeplacerPieceMsg msg);
	void TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1);
	
};

#endif
