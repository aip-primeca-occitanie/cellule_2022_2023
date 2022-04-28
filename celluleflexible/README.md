# TER atelier flexible ou cellule flexible

Petit guide pour lancer le projet sur votre machine : (Ubuntu 20.04, ROS Noetic)

Il est toujours bon de faire un:
```
		sudo apt-get update
```
Installer la librairie modbus avec: 
```
		sudo apt-get install libmodbus-dev
```
Cloner la branche master de ce repository :
(si git n'est pas installé, vous pouvez toujours télécharger le projet en zip et en extraire le code)
```
		git clone https://gitlab.enseeiht.fr/sandra/TERcelluleflexible.git
```
Compiler les packages ros en se plaçant dans le dossier ros_ws et en effectuant :
```
		catkin_make
```
Pour lancer le projet avec les fichiers launch (ancienne version):
Sourcez les fichiers compilés (depuis le dossier ros_ws):
```
		source devel/setup.bash
```
Exécuter le launch.sh à la racine du projet :
```
		cd ..
		./launch.sh
```
Si cela ne marche pas, essayer clic droit sur le fichier launch.sh -> Propriétés, onglet Permissions et cocher la case "Autoriser l'exécution du fichier comme un programme".

En résumé (pour copier/coller dans un terminal):
```
		git clone https://gitlab.enseeiht.fr/sandra/TERcelluleflexible.git
		cd ros_ws
		source devel/setup.bash
		catkin_make
		cd ..
		./launch.sh
```
Pour lancer le projet avec la nouvelle interface :
Pour installer catmux :
```
	  sudo apt-get install tmux
	  pip3 install --user catmux
	  mv Bureau/Cellule_2022_copelia/celluleflexible/catmux/session.py .local/lib/python3.8/site-packages/catmux/session.py

```
Pour lancer le projet avec catmux
```
	  cd ~/Bureau/Cellule_2022_copelia/celluleflexible/catmux/config/
```
Pour lancer la session du projet:
```
 	catmux_create_session projet.yaml
```
Si on veut spécifier le nombre de robots (remplacer ? par le nombre de robots souhaités 1 à 4 max):
```
	catmux_create_session --overwrite nb_robots=? projet.yaml
```
Pour quitter la session:
```
 	tmux kill-session
 	raccourci (ctrl+b -> shift+k -> y)
```
Pour revenir au lancement du projet avec launchs :
```
 	Décommenter ligne 108-109 du fichier commande_locale.cpp 
 	dans Cellule_2022_copelia/celluleflexible/ros_ws/src/Automate/commande_locale/src
```

#[mon lien](http://....)
          
         
