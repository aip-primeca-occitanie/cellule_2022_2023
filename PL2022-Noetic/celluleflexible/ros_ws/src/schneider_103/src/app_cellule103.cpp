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

#include "cellule_tp_103.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <stdlib.h>
#include <string.h>

using namespace std;

// Déclaration des adresses modbus correpondant à la mémoire partagée de l'automate
int ST6 = 0;
int ST7 = 1;
int ST18 = 2;
int ST19 = 3;
int R3D = 4;
int R4D = 5;
int R9D = 6;
int R10D = 7;
int R3G = 8;
int R4G = 9;
int R9G = 10;
int R10G = 11;
int D3 = 12;
int D4 = 13;
int D9 = 14;
int D10 = 15;
int V3 = 16;
int V4 = 17;
int V9 = 18;
int V10 = 19;
int PS7= 20;
int PS18 = 21;
int PS19 = 22;
int D3D = 23;
int D4D = 24;
int D9D = 25;
int D10D = 26;
int D3G = 27;
int D4G = 28;
int D9G = 29;
int D10G = 30;
int CP3 = 31;
int CP8 = 32;



// déclaration des informations de bloc
char bloc_nom[10];



// fonction d'écriture lecture d'un bloc élementaire de scénario 
void lect_ecrit_autom(vector<vector<int>> bloc, std::string nom_bloc,Cellule_tp cellule)

{
		std::cout << "Ecriture du " << nom_bloc <<"\n"<<std::endl;   
        	cellule.write(bloc);
		std::cout << nom_bloc << "écrit\n" << std::endl;
		cellule.read();
		std::cout << "Lecture des informations automate effectuées\n" << std::endl;
}








/* fonction d'attente conditionnel 
void attente_conditionnel(std::string condition, lect_cellule Cellule)

{

        while(condition) {Cellule.read();}
        std::cout << "condition d'attente ok\n" << std::endl;	
}
*/
int main(int argc, char **argv)
{
	
	// Creation du noeud ROS
	std::cout << "initialisation de ros\n" << std::endl;
	ros::init(argc, argv, "app_cellule103");
	ROS_DEBUG_NAMED("verification de code", "Demarrage");
    ROS_DEBUG_STREAM_NAMED("verification de code", "demarrage");
	ros::NodeHandle noeud;
	ros::Rate loop_rate(25);


	
	// Importation des fonctions de contrôle
	printf("lancement du noeud de contrôle\n");
	Cellule_tp cellule_103(noeud);
	printf("noeud lancée\n");
	if (ros::ok())
		printf("ros ok\n");
	else
		printf("ros non ok\n");
	
		while (ros::ok())
	{
		cellule_103.read();

		ros::spinOnce();
		loop_rate.sleep();
		

       /* lect_ecrit_autom(bloc1 , "Bloc 1 ", cellule);

		while(!cellule.D1D && !cellule.D2D && !cellule.D11G && !cellule.D12G) {cellule.read();}
        std::cout << "condition d'attente ok\n" << std::endl;
		
		lect_ecrit_autom(bloc2 , "Bloc 2 ", cellule);
	   // Permet laisser du temps système 
		ros::Duration(2).sleep();*/

		//lect_ecrit_autom(bloc3 , "Bloc 3 ", cellule);
        		
		/*while(!cellule.D1G && !cellule.D2G && !cellule.D11D && !cellule.D12D) {cellule.read();}
        std::cout << "condition d'attente ok\n" << std::endl;
		
		lect_ecrit_autom(bloc2 , "Bloc 2 ", cellule);
		ros::Duration(2).sleep();*/ 

	}
	
	return 0;
}

// écriture lecture d'un bloc de scénario
		/*std::cout << "Ecriture du bloc 2 \n" << std::endl;
		cellule.write(bloc2);
      	std::cout << "bloc 2 écrit \n" << std::endl;*/
