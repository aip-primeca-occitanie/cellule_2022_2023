/*###########################################################################
## ___ __ __                      __                      ___              ##
##  | |_ |__)   /\ |_ _ |o _  _  |_ | _   o|_ | _    __    |    |_ _       ##
##  | |__| \   /--\|_(-`||(-`|   |  |(-`><||_)|(-`         | |_||_(_)      ##
##                                                                         ##
############################################################################*/
 
/*!
 * \file Tuto_Basique.main_commande.cpp
 * \brief code correspondant au tuto du sujet de TER atelier flexible
 * \author Team Tuto_Basique (N7 2020-2021)
 * \version 0.1
 */

#include "capteurs.h"
#include "actionneurs.h"
#include "commande.h"
#include "RobotsInterface.h"
#include "AigsInterface.h"
#include <ros/ros.h>
#include <unistd.h>

using namespace std;

#include "prodconfig/productconfig.h"

#define RESET   "\033[0m"
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */

#define PlaceFin 1000 /* Marquage à ne pas dépasser */

int M[PlaceFin+1];

/* *****************************************************************
///////////  | Exemple configuration produits : début |  //////////
 ******************************************************************* */

// type de produit : séquence de POSTES : durée par poste : nombre de produits
// 2 : 1 4 : 4 5 : 2
// 6 : 7 6 5 : 3 6 3 : 1

/*
const vector<int> Prod_type{    2,
                                6};

const vector<int> Prod_qte{ 2,
                            1};

const vector<vector<int>> Prod_seqdeposte{  { POSTE_1, POSTE_4 },
                                            { POSTE_7, POSTE_6, POSTE_5 }};

const vector<vector<int>> Prod_dureeparposte{   { 4, 5 },
                                                { 3, 6, 3 }};
*/
/* ********************************************************
///////////  | Exemple configuration produits : fin |  //////////
 ******************************************************** */



/////////////////////////////////////////////////////////////////////////
//////////////////// | DEBUT DECLARE ETU | /////////////////
/////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////
/////////////////////  |  FIN DECLARE ETU  |   /////////////////
/////////////////////////////////////////////////////////////////////////


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
    
    int nbNavettes=0;//Mettre 0 pour demander a l'utilisateur
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
    

    cmd.Initialisation();
    for(int i=0;i<PlaceFin;i++) M[i]=0;

    /* *************************************************
    ////// | MARQUAGE INITIAL | ////////
    ************************************************* */
    M[0]=1;
    display();
    
    ///////////////////////////////////////////////////////////////////
    ///////////////////// | DEBUT INIT ETU | ///////////////////
    ///////////////////////////////////////////////////////////////////
    
    
    ///////////////////////////////////////////////////////////////////
    /////////////////////  |  FIN INIT ETU  |  ////////////////////
    ///////////////////////////////////////////////////////////////////

    while (ros::ok())
    {
        // Seulement si la simulation est en cours
        if(cmd.getPlay()==true)
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////// | DEBUT PETRI  ETU | /////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if(M[0])
            {
            /*!
                        * \b T1: init aiguillages et produit
                        * \arg positionnement des aiguillages et ajout des produits
                        * \arg \b Precondition: M[0]
                        * \arg \b Postcondition: M[2]++
                        */
                M[0]--;


                aiguillage.Gauche(3);
                aiguillage.Gauche(10);
                aiguillage.Gauche(11);
                aiguillage.Gauche(12);
                aiguillage.Gauche(1);

                robot.AjouterProduit(Prod_seqdeposte[0][0], Prod_type[0]);
                robot.FaireTache(Prod_seqdeposte[0][0], Prod_dureeparposte[0][0]);
                cout << "duree poste=" << Prod_dureeparposte[0][0] << endl;

                M[2]++;
                display();
            }
            if(M[2] && capteur.get_PS(6))
            {
                /*!
                        * \b T2: aiguillage A02 mise en place
                        * \arg  courte description
                        * \arg \b Precondition: M[2] && capteur.get_PS(6)
                        * \arg \b Postcondition: M[3]++
                        */
                M[2]--;

                aiguillage.Gauche(2);

                M[3]++;
                display();
            }
            if(M[3] && robot.TacheFinie(Prod_seqdeposte[0][0]))
            {
            /*!
                        * \b T4: positionnement aiguillages vers prochain poste 3
                        * \arg courte description
                        * \arg \b Precondition: M[3] && robot.TacheFinie(Prod_seqdeposte[0][0])
                        * \arg \b Postcondition: M[4]++
                        */
                M[3]--;

                aiguillage.Droite(11);
                aiguillage.Droite(12);
                aiguillage.Droite(01);

                cmd.Stop_PS(22);

                

                M[4]++;
                display();
            }
            if(M[4] && capteur.get_PS(22))
            {
            /*!
                        * \b T3: piece de poste 2 à navette
                        * \arg deplacement de piece
                        * \arg \b Precondition: M[4] && capteur.get_PS(22)
                        * \arg \b Postcondition: M[PlaceAval]++; M[PlaceAvalBis]
                        */
                M[4]--;

                robot.DeplacerPiece(ROBOT_1, 4, 3);

                M[5]++;
                display();
            }
            if(M[5] && robot.FinDeplacerPiece(ROBOT_1))
            {
            /*!
                        * \b T5: faire repartir navette PS22
                        * \arg navette repart de PS22, PS3 stop activé
                        * \arg \b Precondition: M[5] && robot.FinDeplacerPiece(ROBOT_1)
                        * \arg \b Postcondition: M[6]++
                        */
                M[5]--;

                cmd.Ouvrir_PS(22);
                cmd.Stop_PS(2);


                M[6]++;
                display();
            }
            if(M[6] && capteur.get_PS(2))
            {
            /*!
                        * \b T6: deplacement piece navette poste
                        * \arg deplacement piece depuis navette sur PS3 vers poste 3
                        * \arg \b Precondition: M[6] && capteur.get_PS(3)
                        * \arg \b Postcondition: M[7]++
                        */
                M[6]--;

                robot.DeplacerPiece(ROBOT_2, 2, 1);

                M[7]++;
                display();
            }
            if(M[7] && robot.FinDeplacerPiece(ROBOT_2))
            {
            /*!
                        * \b T7: lancer tache sur poste 3
                        * \arg lancer tache sur POSTE_3
                        * \arg \b Precondition: M[7] && robot.FinDeplacerPiece(ROBOT_2)
                        * \arg \b Postcondition: M[8]++
                        */
                M[7]--;


                robot.Evacuer();
                M[8]++;
                display();
            }
            if(M[8])
            {
            /*!
                        * \b T8: evacuer et fin rdp
                        * \arg evacuer et fin du rdp
                        * \arg \b Precondition: M[8] && robot.TacheFinie(Prod_seqdeposte[0][1])
                        * \arg \b Postcondition: M[PlaceFin]++
                        */
                M[8]--;


                M[PlaceFin]++;
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
