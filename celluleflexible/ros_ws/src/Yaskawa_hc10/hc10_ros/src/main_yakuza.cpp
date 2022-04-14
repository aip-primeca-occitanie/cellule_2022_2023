/*
################
#### Entete ####
################
#
# This project has received funding from the European Union s Horizon 2020 research and innovation programme under grant agreement No 732287.
# The RIO project starts in Jan 2020 and ends in Dec 2020.
#
# Description du programme
#  -> Creation de la classe Recept : permet de souscrire à l un des trois topics publies par le noeud ROBOT et met a jour ses variables publiques 
#  (qui sont les memes que la classe Etat) avec les informations recues
#
# Entree
#  -> aucune
#
# Sortie 
#  -> aucune
#
# Historique
#  -> Creation: 29/09/2020, AIP PRIMECA Occitanie, Nathan MORET
#  -> Ajout de la fonction lect_ecrit_autom et commentaire de code 15/11/2021, AIP PRIMECA OCCITANIE, Fabien Marco
######################
#### Fin d entete ####
######################
*/

#include "Yakuza_tp.h"
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
	ros::init(argc, argv, "main_yaskawa");
	ROS_DEBUG_NAMED("verification de code", "Demarrage");
    	ROS_DEBUG_STREAM_NAMED("verification de code", "demarrage");
	ros::NodeHandle noeud;
	ros::Rate loop_rate(25);


	
	// Importation des fonctions de contrôle
	printf("lancement du noeud de contrôle\n");
	Yakuza_tp yaskawa(noeud);
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


