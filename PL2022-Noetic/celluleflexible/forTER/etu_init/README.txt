
Toutes les instructions données supposent que vous vous trouvez à la racine du dossier "etu"

///////////////////////////////////////////////////////////////////

A FAIRE UNE SEULE FOIS LORS DE LA PREMIERE SEANCE DE TER

///////////////////////////////////////////////////////////////////


Renommer le fichier principal XXX_XXX.main_commande.cpp en y mettant vos initiales (majuscule, sans espace, sans accent, sans symboles spéciaux):
	mv XXX_XXX.main_commande.cpp VOS_INITIALES.main_commande.cpp




/////////////////////////////////////////////////////////////////////////////////////

POUR COMPILER ET LANCER LA SIMULATION GENEREE PAR VOTRE FICHIER "VOS_INITIALES.main_commande.cpp" 

/////////////////////////////////////////////////////////////////////////////////////


Pour compiler et exécuter votre fichier "VOS_INITIALES.main_commande.cpp" en considérant en données de production le fichier NOM_FICHIER_DATA.prodconfig.h contenu dans le dossier dataprodconfig, il suffit de faire:

	./script1_CompileandRun.sh VOS_INITIALES NOM_FICHIER_DATA

Par exemple, si vos initiales sont SUN_SC (et donc le fichier commande associé est SUN_SC.main_commande.cpp), et que le fichier data à considérer est data4_2_2_n0.prodconfig.h, alors l'instruction pour compiler et exécuter sera : 

	./script1_CompileandRun.sh SUN_SC data4_2_2_n0


/////////////////////////////////////////////////////////////////////////////////////

POUR IMPORTER LES RESULTATS DE LA SIMULATION (après avoir choisi "4-Fin programme" dans la fenêtre XTERMINAL)

/////////////////////////////////////////////////////////////////////////////////////

	./script2_GetSimuOutput.sh SUN_SC data4_2_2_n0

Après chaque exécution sans bug de la simulation et après l'import des résultats, un fichier log nommé "VOS_INITIALES_NOM_FICHIER_DATA.log.txt" doit apparaitre dans le dossier "output". 
Si ce fichier n'existe pas, c'est qu'il y a eu une erreur lors de votre simulation.



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
	./script1_CompileandRun.sh TeamSALLAG data4_3_2_n0
	./script2_GetSimuOutput.sh TeamSALLAG data4_3_2_n0
	./script3_Validation.sh TeamSALLAG data4_3_2_n0
	./script4_Documentation.sh TeamSALLAG
	rm TeamSALLAG.main_commande.cpp



