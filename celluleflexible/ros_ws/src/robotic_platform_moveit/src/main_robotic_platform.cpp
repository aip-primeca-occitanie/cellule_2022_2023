/*
################
#### Entete ####
################
#
#
# Description du programme
#  -> Realise la connexion entre la simulation CoppeliaSim et la simulation Rviz des robots
#
# Entree
#  -> aucune
#
# Sortie 
#  -> aucune
#
# Historique
#  -> Creation: 04/05/2022, AIP PRIMECA Occitanie, Alexandre Belveze
######################
#### Fin d entete ####
######################
*/

#include "robotic_platform.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <stdlib.h>
#include <string.h>

using namespace std;


int main(int argc, char **argv)
{
	
	// Creation du noeud ROS
	std::cout << "initialisation de ros\n" << std::endl;
	string name="robot_";
	name.append(argv[1]);
	ros::init(argc,argv,name);
	ROS_DEBUG_NAMED("verification de code", "Demarrage");
    ROS_DEBUG_STREAM_NAMED("verification de code", "demarrage");
	ros::NodeHandle noeud;
	ros::Rate loop_rate(25);


	
	// Importation des fonctions de contrôle
	printf("lancement du noeud de contrôle\n");
	robotic_platform myrobot(noeud,argv[1]);
	printf("noeud lancée\n");
	if (ros::ok())
		printf("ros ok\n");
	else
		printf("ros non ok\n");
	
		while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		


	}
	
	return 0;
}


