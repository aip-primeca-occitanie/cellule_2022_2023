
Toutes les instructions données supposent que vous vous trouvez à la racine du dossier "etu"

///////////////////////////////////////////////////////////////////

A FAIRE UNE SEULE FOIS LORS DE LA PREMIERE SEANCE DE TER

///////////////////////////////////////////////////////////////////


Renommer le fichier principal XXX_XXX.main_commande.cpp en y mettant vos initiales (majuscule, sans espace, sans accent, sans symboles spéciaux):
	mv XXX_XXX.main_commande.cpp VOS_INITIALES.main_commande.cpp



///////////////////////////////////////////////////////////////////

A FAIRE A CHAQUE NOUVELLE OUVERTURE D'UN NOUVEAU TERMINAL (ou en cas de fermeture incorrecte d'une simulation)

///////////////////////////////////////////////////////////////////


Taper les instructions suivantes directement dans le terminal (ces instructions sont aussi contenues dans le fichier Initialisation.sh, l'exécution de ce fichier ne semble pas se réaliser les fonctions souhaitées): 

	source /opt/ros/melodic/setup.bash
	cd ../celluleflexible/ros_ws
	catkin_make
	source devel/setup.bash


/////////////////////////////////////////////////////////////////////////////////////

POUR COMPILER LA SIMULATION GENEREE PAR VOTRE FICHIER "VOS_INITIALES.main_commande.cpp" 

/////////////////////////////////////////////////////////////////////////////////////


Pour compiler "VOS_INITIALES.main_commande.cpp" en considérant en données de production le fichier NOM_FICHIER_DATA.prodconfig.h contenu dans le dossier dataprodconfig, il suffit de faire:

	./script0_Compile.sh VOS_INITIALES NOM_FICHIER_DATA

Par exemple, si vos initiales sont SUN_SC (et donc le fichier commande associé est SUN_SC.main_commande.cpp), et que le fichier data à considérer est data4_2_2_n0.prodconfig.h, alors l'instruction pour compiler sera : 

	./script0_Compile.sh SUN_SC data4_2_2_n0


/////////////////////////////////////////////////////////////////////////////////////

POUR LANCER LA SIMULATION GENEREE PAR VOTRE FICHIER "VOS_INITIALES.main_commande.cpp" 

/////////////////////////////////////////////////////////////////////////////////////

APRES avoir vérifié qu'aucune erreur de compilation ne s'est précédemment produite:
Pour lancer la simulation résultante, exécuter:

	./script1_Run.sh 


/////////////////////////////////////////////////////////////////////////////////////

POUR IMPORTER LES RESULTATS DE LA SIMULATION (après avoir choisi "4-Fin programme" dans la fenêtre XTERMINAL)

/////////////////////////////////////////////////////////////////////////////////////

	./script2_GetSimuOutput.sh VOS_INITIALES NOM_FICHIER_DATA

Après chaque exécution sans bug de la simulation et après l'import des résultats avec le script2, un fichier log nommé "VOS_INITIALES_NOM_FICHIER_DATA.log.txt" doit apparaitre dans le dossier "output". 
Si ce fichier n'existe pas, c'est qu'il y a eu une erreur lors de votre simulation.

Par exemple, si vos initiales sont SUN_SC (et donc le fichier commande associé est SUN_SC.main_commande.cpp), et que le fichier data à considérer est data4_2_2_n0.prodconfig.h, alors l'instruction pour compiler sera : 

	./script2_GetSimuOutput.sh SUN_SC data4_2_2_n0



/////////////////////////////////////////////////////////////////////////////////////

POUR VERIFIER LA VALIDITE DE L'EXECUTION DE LA SIMULATION PAR RAPPORT AUX DONNEES DE PRODUCTION

/////////////////////////////////////////////////////////////////////////////////////

Le fichier log précédemment généré permet de vérifier la validité de votre simulation par rapport aux données d'entrée.

Pour ce faire, depuis la racine du dossier "etu", vous pouvez exécuter:

	./script3_Validation.sh VOS_INITIALES NOM_FICHIER_DATA



////////////////////////////////////////////////////////////////////////////////////////

POUR GENERER LA DOCUMENTATION ASSOCIEE A VOTRE FICHIER "VOS_INITIALES.main_commande.cpp" 

////////////////////////////////////////////////////////////////////////////////////////

Pour générer la documentation associée à votre code, depuis la racine du dossier "etu", exécuter:

	./script4_Documentation.sh VOS_INITIALES

La documentation devrait alors être générée dans le dossier output/docfromcpp




////////////////////////////////////////////////////////////////////////////////////////

VERIFIER QUE LA SIMULATION COPPELIA FONCTIONNE BIEN (avec le code exemple de TeamSALLAG)

////////////////////////////////////////////////////////////////////////////////////////

Le fichier TeamSALLAG.main_commande.cpp contenu dans le dossier exemple peut permettre de vérifier le bon fonctionnement de la simulation coppelia :

	cp exemple/TeamSALLAG.main_commande.cpp .
	./script0_Compile.sh TeamSALLAG data4_3_2_n0
	./script1_Run.sh
	./script2_GetSimuOutput.sh TeamSALLAG data4_3_2_n0
	./script3_Validation.sh TeamSALLAG data4_3_2_n0
	./script4_Documentation.sh TeamSALLAG
	rm TeamSALLAG.main_commande.cpp



