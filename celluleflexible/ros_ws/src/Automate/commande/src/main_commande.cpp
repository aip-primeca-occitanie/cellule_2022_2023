/*########################################################################
##  ______                        _____ ___    __    __    ___   ______ ##
## /_  __/__  ____ _____ ___     / ___//   |  / /   / /   /   | / ____/ ##
##  / / / _ \/ __ `/ __ `__ \    \__ \/ /| | / /   / /   / /| |/ / __   ##
## / / /  __/ /_/ / / / / / /   ___/ / ___ |/ /___/ /___/ ___ / /_/ /   ##
##/_/  \___/\__,_/_/ /_/ /_/   /____/_/  |_/_____/_____/_/  |_\____/    ##
######### Steve - Anthony - Lucie - Lucas - Antonin - Guillaume ##########
##########Croce - Favier - Ricart - Veit - Messioux - Auffray-Amen########
##########################################################################
### As a wise man once said : "Follow the white rabbit with hhbbgd...."###
##########################################################################*/
 
//#include "capteurs.h"
#include "capteurs.h"
#include "actionneurs.h"
#include "commande.h"
#include "RobotsInterface.h"
#include "AigsInterface.h"
#include <ros/ros.h>
#include <unistd.h>

using namespace std;

#define RESET   "\033[0m"
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */

#define PlaceFin 1000 /* Marquage à ne pas dépasser */

int M[PlaceFin+1];
int cpt =0;

// Pour l'affichage //
void display()
{
	cout << endl;
	for (int i=0;i<=PlaceFin;i++)
	{
		if(i==0)
			cout << "Marquage : ";

		if(M[i]>0)
			cout<<BOLDRED<<"M["<<i<<"]="<<M[i]<<RESET<<", ";
		if(M[i]<0)
			cout<<BOLDGREEN<<"M["<<i<<"]="<<M[i]<<RESET<<", ";
	}
	cout<<endl<<endl;
}

void ShutdownCallback(const std_msgs::Byte::ConstPtr& msg)
{
		ros::shutdown();
}

int main(int argc, char **argv)
{
    /* *************************************************
	///////////  | Debut du Petri plus bas |  //////////
     ************************************************* */


	ros::init(argc, argv, "commande");
	ros::NodeHandle noeud;

	ros::Publisher pub_spawnShuttles = noeud.advertise<std_msgs::Int32>("/commande_locale/nbNavettes",10);
	ros::Subscriber sub_shutdown = noeud.subscribe("/commande_locale/shutdown",10,&ShutdownCallback);

	int nbRobot=atoi(argv[1]);

	Commande cmd(noeud,argv[0]);
	RobotsInterface robot(noeud,nbRobot);
	AigsInterface aiguillage(noeud);
	Capteurs capteur(noeud);

	ros::Rate loop_rate(25); //fréquence de la boucle

	// On attend la fin de l'initialisation des robots
	while(!robot.RobotInitialise(1) || !robot.RobotInitialise(2))
	{
		ros::spinOnce();
		loop_rate.sleep();
	}	
	while(nbRobot==4 && (!robot.RobotInitialise(3) || !robot.RobotInitialise(4)))
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

    /* *************************************************
	// | Creation des Navettes | //
     ************************************************* */

    vector<int> listeNavettes{1};
    // vector<int> listofnavettes;
    // vector<int> listofnavettes{1, 4};
    // vector<int> listofnavettes{0, 1, 2, 3, 4, 5, 6};
    
    while(listeNavettes.size()<1||listeNavettes.size()>7)
    {
        int nbNavettes=1;
        cout << "Combien voulez vous de navettes ? [1..6]" << endl;
        cin >> nbNavettes;
        if(cin.fail())
        {
            cout << endl << " [Erreur mauvais choix ..]" << endl;
            cin.clear();
            cin.ignore(256,'\n');
        }
        else
        {
            for (int i=0; i < nbNavettes; i++)
            {
                listeNavettes.push_back(i+1); // SUN avoid navet 0 because its initial position is different
            }
        }
    }
    
    
    
    /*SUN
	int nbNavettes=1;//Mettre 0 pour demander a l'utilisateur
	while(nbNavettes<1||nbNavettes>6)
	{
		cout << "Combien voulez vous de navettes ? [1..6]" << endl;
		cin >> nbNavettes;
		if(cin.fail())
		{
			cout << endl << " [Erreur mauvais choix ..]" << endl;
			cin.clear();
			cin.ignore(256,'\n');
		}
	}
    
	std_msgs::Int32 msg_nbNavettes;
	msg_nbNavettes.data=nbNavettes;
	pub_spawnShuttles.publish(msg_nbNavettes);
    */
    
    
    for (int i=0; i < listeNavettes.size(); i++)
    {
        std_msgs::Int32 msg_nbNavettes;
        msg_nbNavettes.data=listeNavettes[i];
        pub_spawnShuttles.publish(msg_nbNavettes);
    }
    

	cmd.Initialisation();
	for(int i=0;i<PlaceFin;i++) M[i]=0;

    /* *************************************************
	////// | MARQUAGE INITIAL | ////////
    ************************************************* */
	M[0]=1;
	M[1]=1;
	display();
    
    ////////////////////////////////////////////////////////////////////
    /////////////// | DEBUT INIT ETU | //////////////
    ////////////////////////////////////////////////////////////////////
    
    
    ////////////////////////////////////////////////////////////////////
    ////////////////  |  FIN INIT ETU  |   //////////////
    ////////////////////////////////////////////////////////////////////

	while (ros::ok())
	{
		// Seulement si la simulation est en cours
		if(cmd.getPlay()==true)
		{
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////// | DEBUT PETRI  ETU | /////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/*aiguillage.Droite(1);
            	aiguillage.Droite(2);
            	aiguillage.Droite(11);	
		aiguillage.Droite(12);*/

		/*if (M[0])
		{	
			sleep(10);
			M[0]--;	
			robot.DeplacerPiece(ROBOT_1,1,3);
			M[70]--;	
		}*/
			if(M[0])
			{		
				M[0]--;
				robot.DeplacerPiece(ROBOT_4,1,2);
				M[2]++;
				display();
			}
			// if(M[1])
			// {		
			// 	M[1]--;
			// 	robot.DeplacerPiece(ROBOT_2,1,2);
			// 	M[3]++;
			// 	display();
			// }
		// if(M[10]){
		// 	if(robot.FinDeplacerPiece(ROBOT_3)){
		// 		M[10]--;
		// 		M[11]++;
		// 	}
		// 	display();			
		// }
		// // Si deux tâche R4 a la suite
			if(M[2]){ 
				if(robot.FinDeplacerPiece(ROBOT_4)){
					M[2]--;
					M[9]++;
				}
				display();
			}
			if(M[9]){ 
				if(robot.FinDeplacerPiece(ROBOT_4)){
					cpt ++; // compteur pour gérer plusieurs tâches R4 successives
				}
				if(!robot.FinDeplacerPiece(ROBOT_4) && cpt != 0){
					M[9]--;
					cpt = 0; // fin deplacierpiece remis a zero, on peut passer a l'action suivante
					robot.DeplacerPiece(ROBOT_4,3,4);
					M[30]++;
				}
				display();
			}
			if(M[30]){ 
				if(robot.FinDeplacerPiece(ROBOT_4)){
					M[30]--;
					M[PlaceFin]++;
				}
				display();
			}
		// if(M[30] & M[11]){ 
		// 	if(robot.FinDeplacerPiece(ROBOT_4)){
		// 		cpt ++; // compteur pour gérer plusieurs tâches R4 successives
		// 	}
		// 	if(!robot.FinDeplacerPiece(ROBOT_4) && cpt != 0){
		// 		M[30]--;
		// 		M[11]--;
		// 		cpt = 0; // fin deplacierpiece remis a zero, on peut passer a l'action suivante
		// 		robot.DeplacerPiece(ROBOT_4,1,3);
		// 		M[31]++;
		// 	}
		// 	display();
		// }
		// if(M[31]){ 
		// 	if(robot.FinDeplacerPiece(ROBOT_4)){
		// 		cpt ++; // compteur pour gérer plusieurs tâches R4 successives
		// 	}
		// 	if(!robot.FinDeplacerPiece(ROBOT_4) && cpt != 0){
		// 		M[31]--;
		// 		cpt = 0; // fin deplacierpiece remis a zero, on peut passer a l'action suivante
		// 		robot.DeplacerPiece(ROBOT_4,3,1);
		// 		M[54]++;
		// 	}
		// 	display();
		// }
			// if(M[3]){ 
			// 	if(robot.FinDeplacerPiece(ROBOT_2)){
			// 		M[3]--;
			// 		M[5]++;
			// 	}
			// 	display();
			// }
			// if(M[4]&M[5]){
			// 	M[4]--;
			// 	M[5]--;
			// 	M[PlaceFin]++;
			// 	display();
			// }
				
            
            
            
            
            
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////// | Place de fin de Petri ETU | //////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if(M[PlaceFin]){
				display();
				cout << endl << BOLDCYAN << " --[PETRI TERMINE]--" << RESET << endl;
				cmd.FinPetri();
				while(ros::ok()){
					ros::spinOnce();
					loop_rate.sleep();
				}
			}
		}
		ros::spinOnce(); //permet aux fonction callback de ros dans les objets d'êtres appelées
		loop_rate.sleep(); //permet de synchroniser la boucle while. Il attend le temps qu'il reste pour faire le 25Hz (ou la fréquence indiquée dans le loop_rate)
	}

	return 0;
}
