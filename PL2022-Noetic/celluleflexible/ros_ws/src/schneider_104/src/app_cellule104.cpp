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

#include "cellule_tp_104.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <stdlib.h>
#include <string.h>

using namespace std;

// Déclaration des adresses modbus correpondant à la mémoire partagée de l'automate
int ST8 = 0;
int ST9 = 1;
int ST10 = 2;
int ST11 = 3;
int ST12 = 4;
int ST13 = 5;
int ST14 = 6;
int ST15 = 7;
int ST16 = 8;
int ST17 = 9;
int R5D = 10;
int R6D = 11;
int R7D = 12;
int R8D = 13;
int R5G = 14;
int R6G = 15;
int R7G = 16;
int R8G = 17;
int PI3 = 18;
int PI4 = 19;
int PI5 = 20;
int PI6 = 21;
int D5 = 22;
int D6 = 23;
int D7 = 24;
int D8 = 25;
int V5 = 26;
int V6 = 27;
int V7 = 28;
int V8 = 29;
int PS8 = 50;
int PS9 = 51;
int PS10 = 52;
int PS11 = 53;
int PS12 = 54;
int PS13 = 55;
int PS14 = 56;
int PS15 = 57;
int PS16 = 58;
int PS17 = 59;
int D5D = 60;
int D6D = 61;
int D7D = 62;
int D8D = 63;
int D5G = 64;
int D6G = 65;
int D7G = 66;
int D8G = 67;
int CPI3 = 68;
int CPI4 = 69;
int CPI5 = 70;
int CPI6 = 71;
int CP4 = 72;
int CP5 = 73;
int CP6 = 74;
int CP7 = 75;


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
	ros::init(argc, argv, "app_cellule104");
	ROS_DEBUG_NAMED("verification de code", "Demarrage");
    ROS_DEBUG_STREAM_NAMED("verification de code", "demarrage");
	ros::NodeHandle noeud;
	ros::Rate loop_rate(25);


	
	// Importation des fonctions de contrôle
	printf("lancement du noeud de contrôle\n");
	Cellule_tp cellule_104(noeud);
	printf("noeud lancée\n");
	if (ros::ok())
		printf("ros ok\n");
	else
		printf("ros non ok\n");
	
		while (ros::ok())
	{
		cellule_104.read();

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
