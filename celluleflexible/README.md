# TER atelier flexible ou cellule flexible

Petit guide pour lancer le projet sur votre machine : (Ubuntu 16, ROS Kinetic)

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
          
En résumé (pour copier/coller dans un terminal):

          git clone https://gitlab.enseeiht.fr/sandra/TERcelluleflexible.git
          cd ros_ws
          source devel/setup.bash
          catkin_make
          cd ..
          ./launch.sh
 
 
          
          
          
