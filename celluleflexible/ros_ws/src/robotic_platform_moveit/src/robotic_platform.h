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
	ros::Publisher pub_robot_;
	ros::Subscriber choixMode_;
	ros::Subscriber robot_;
	std_msgs::Int32 DeplacementRobot_;
	int yaska4Type_;
	int yaska3Type_;
	int kukaType_;
	int staubliType_;
	std::string name_;
	enum Robot_{Coppelia = 0, Rviz, Atelier};
	enum Group_{DN1P = 1, DN2P, DPN1, DPN2};
	
public:
	robotic_platform(ros::NodeHandle noeud, std::string name);
	~robotic_platform();
	void RobCallabck(const commande_locale::DeplacerPieceMsg msg);
	void TypeMode(const commande_locale::Msg_ChoixMode::ConstPtr& msg1);
	
};

#endif
