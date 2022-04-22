# TER atelier flexible ou cellule flexible

Petit guide pour lancer le projet sur votre machine : (Ubuntu 20.04, ROS Noetic)

- 0) Il est toujours bon de faire un 

          sudo apt-get update

- 1) Installer la librairie modbus avec 

          sudo apt-get install libmodbus-dev
          
- 2) Cloner la branche master de ce repository :
    (si git n'est pas installé, vous pouvez toujours télécharger le projet en zip et en extraire le code)
    
          git clone https://gitlab.enseeiht.fr/sandra/TERcelluleflexible.git
        
          
- 3) Compiler les packages ros en se plaçant dans ros_ws et en effectuant :

          catkin_make
          
- 4) Sourcez les fichiers compilés (depuis le dossier ros_ws):

          source devel/setup.bash
          
- 5) Exécuter le launch.sh à la racine du projet :

          ./launch.sh
          
  Si cela ne marche pas, essayer clic droit sur le fichier launch.sh -> Propriétés, onglet Permissions
  et cocher la case "Autoriser l'exécution du fichier comme un programme"
  
En résumé (pour copier/coller dans un terminal):

          git clone https://gitlab.enseeiht.fr/sandra/TERcelluleflexible.git
          cd ros_ws
          source devel/setup.bash
          catkin_make
          cd ..
          ./launch.sh
 
Juste faire dans un terminal :
	 cd ~/Bureau/Cellule_2022_copelia/celluleflexible/catmux config
Pour lancer le projet:
 	catmux_create_session example_session.yaml
Pour quitter le projet:
 	tmux kill-session
 
Pour revenir au lancement du projet avec les fichiers launchs :
 	Décommenter ligne 108-109 du fichier commande_locale.cpp 
 	dans Cellule_2022_copelia/celluleflexible/ros_ws/src/Automate/commande_locale/src
          
         
