# TER atelier flexible ou cellule flexible sous ubuntu 20 avec ROS Noetic

Dans le répertoire de travail : 

          git clone -b Noetic https://github.com/Yacoubo/PL2022.git

Il faut renommer le répertoire PL2022 en TERcelluleflexible

Puis lancement du test

          commande : cd TERcelluleflexible/celluleflexible/ros_ws
          Supprimer les dossiers build et devel
          commande : cd src
          Supprimer le fichier CMakeLists.txt
          commande : cd ..
          commande : catkin_make
          commande source devel/setup.bash
          commande : cd ..
          commande ./launch.sh


# PARTIE ETUDIANTE

Dans le dossier TERcelluleflexible il devrait y avoir deux dossier: celluleflexible et etu (si ce dossier n'existe pas, se placer à la racine du dossier TERcelluleflexible et exécuter l'instruction suivante : cp celluleflexible/forTER/etu_init etu)

Se placer dans le dossier etu (c'est dans ce dossier que s'effectueront toutes les manipulations du TER):
    
          cd etu

 S'assurer que les différents scripts sont bien executables:
 
          (chmod -R 777 etu) ou
          chmod +x script0_Compile.sh
          chmod +x script1_Run.sh
          chmod +x script2_GetSimuOutput.sh
          chmod +x script3_Validation.sh
          chmod +x script4_Documentation.sh

A présent vous êtes prêt à commencer le TER, se référer aux fichiers dans le dossier etu pour la suite.


Quelques erreurs lors de l'installation la première fois
s'assurer d'avoir fait dans le terminal: source /opt/ros/melodic/setup.bash
s'assurer que xterm est installé: sudo apt install xterm
recupérer coppelia sim ubuntu18 et écraser/fusionner avec le dossier actuel
depuis le dossier ros_ws faire source devel/setup.bash


