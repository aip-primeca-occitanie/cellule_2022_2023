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
			cmd.Ouvrir_PS(1);
			cmd.Ouvrir_PS(2);
			cmd.Ouvrir_PS(3);
			cmd.Ouvrir_PS(4);
			cmd.Stop_PS(5);
			cmd.Ouvrir_PS(6);
			cmd.Ouvrir_PS(7);
			cmd.Ouvrir_PS(8);
			cmd.Ouvrir_PS(9);	
			cmd.Ouvrir_PS(10);
			cmd.Ouvrir_PS(11);
			cmd.Ouvrir_PS(12);
			cmd.Ouvrir_PS(13);
			cmd.Ouvrir_PS(14);
			cmd.Ouvrir_PS(15);
			cmd.Ouvrir_PS(16);
			cmd.Ouvrir_PS(17);
			cmd.Ouvrir_PS(18);
			cmd.Ouvrir_PS(19);	
			cmd.Ouvrir_PS(20);
			cmd.Ouvrir_PS(21);
			cmd.Ouvrir_PS(22);
			cmd.Ouvrir_PS(23);
			cmd.Ouvrir_PS(24);			
			
			M[13]++;

			display();
  	
		}
		if(M[13]){
			M[13]--;
			
			aiguillage.Gauche(3);
			aiguillage.Gauche(10);
			M[15]++;

		}

		if(M[15])
		{	

			if(capteur.get_PS(5)){
				
				M[15]--;
				aiguillage.Droite(2);
				aiguillage.Droite(11);
				aiguillage.Droite(1);
				aiguillage.Droite(12);
				M[14]++;
			}

			display();
  	
		}
		if(M[14])
		{
			if(capteur.get_DD(1) && capteur.get_DD(2) && capteur.get_DD(11) && capteur.get_DD(12))
			{	
				M[14]--;
				cmd.Ouvrir_PS(5);
				cmd.Stop_PS(21);
				M[50]++;
			}
			display();
		}
		if(M[50])
		{
			if(capteur.get_PS(21))
			{
				M[50]--;
				robot.DeplacerPiece(ROBOT_1,2,4);				
				M[2]++;
			}
			
			display();
		}
		if(M[2])
		{
			if(robot.FinDeplacerPiece(ROBOT_1))
			{
				M[2]--;
				aiguillage.Gauche(1);
				aiguillage.Gauche(2);
				aiguillage.Droite(3);	
				aiguillage.Droite(4);
				aiguillage.Droite(5);
				aiguillage.Droite(6);
				aiguillage.Gauche(7);
				aiguillage.Gauche(8);
				aiguillage.Droite(9);
				aiguillage.Droite(11);
				M[3]++;
			}
			display();
		}
		if(M[3])
		{
			if(capteur.get_DG(1) && capteur.get_DG(2) && capteur.get_DD(3) && capteur.get_DD(4) && capteur.get_DD(5) && capteur.get_DD(6) && capteur.get_DG(7) && capteur.get_DG(8) && capteur.get_DD(9) && capteur.get_DD(11))
			{
				M[3]--;
				cmd.Ouvrir_PS(21);
				cmd.Stop_PS(9);
				M[8]++;
			}
			display();
		}		
        	if(M[8]){ 
			if(capteur.get_PS(9))
			{
				M[8]--;
				robot.DeplacerPiece(ROBOT_4,4,2);
				cmd.Stop_PS(10);
				M[9]++;
			}
			display();
		}
		if(M[9]){ 
			if(robot.FinDeplacerPiece(ROBOT_4))
			{
				M[9]--;
				cmd.Ouvrir_PS(9);
				M[10]++;
			}
			display();
		}
		if(M[10]){ 
			if(capteur.get_PS(10))
			{
				M[10]--;
				robot.DeplacerPiece(ROBOT_4,3,4);
				aiguillage.Droite(10);
				cmd.Stop_PS(22);				
				M[51]++;
			}
			display();
		}
		if(M[51]){ 
			if(robot.FinDeplacerPiece(ROBOT_4) && capteur.get_DD(10))
			{
				M[51]--;
				cmd.Ouvrir_PS(10);
				M[52]++;
			}
			display();
		}
		if(M[52]){ 
			if(capteur.get_PS(22))
			{
				M[52]--;
				robot.DeplacerPiece(ROBOT_1,4,3);
				aiguillage.Droite(12);
				aiguillage.Droite(1);
				cmd.Stop_PS(5);
				M[53]++;
			}
			display();
		}	
		if(M[53]){ 
			if(robot.FinDeplacerPiece(ROBOT_1) && capteur.get_DD(12)  && capteur.get_DD(1))
			{
				M[53]--;
				cmd.Ouvrir_PS(22);
				M[54]++;
			}
			display();
		}
		if(M[54]){ 
			if(capteur.get_PS(5))
			{
				M[54]--;
				
				M[PlaceFin]++;
			}
			display();
		}
				
            
            
            
            
            
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////// | Place de fin de Petri ETU | //////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if(M[PlaceFin])
			{
				display();
				cout << endl << BOLDCYAN << " --[PETRI TERMINE]--" << RESET << endl;
				cmd.FinPetri();
				while(ros::ok())
				{
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

