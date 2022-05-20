# Installation du projet de zéro :

- Si vous êtes sur windows, lancer une machine virtuelle Ubuntu 20.04
- Ouvrir un terminal et faire :
```
		sudo apt update
		sudo apt upgrade
```

- Cloner le répertoire de travail sur le bureau :
```
		sudo apt install git
		git clone https://github.com/AlexBelveze/Cellule_2022_copelia.git
```

- Si ros noetic n'est pas installé, suivre la procédure :
```
		http://wiki.ros.org/noetic/Installation/Ubuntu
```

- Ajouter les packages ros suivants :
```
		sudo apt install ros-noetic-moveit
		sudo apt install ros-noetic-industrial-*
```

- Installer la librairie modbus :
```
		sudo apt-get install libmodbus-dev
		sudo pip install pyModbusTCP
```

- Pour installer catmux :
```
		sudo apt-get install tmux
		pip3 install --user catmux
		mv Bureau/Cellule_2022_copelia/celluleflexible/catmux/session.py .local/lib/python3.8/site-packages/catmux/session.py
```
- Une fois l'installation terminé il faut vérifier à la ligne 7 du fichier Cellule_2022_copelia/celluleflexible/catmux/projet.yaml que le chemin d'accès corresponde au votre :
```
	 - cd ~/Desktop/Cellule_2022_copelia/celluleflexible/ros_ws
	 #Desktop doit être remplacé par l'endroit ou vous avez cloner le répertoire	
```

- A cette étape il faut faire :
```
		sudo reboot
```

- Pour compiler le projet :
```
		cd PATH_TO_PROJECT/Cellule_2022_copelia/celluleflexible/ros_ws
		catkin_make
```

- Pour lancer le projet : (par défaut, il se lance avec les 4 robots et les deux plateformes)
```
		cd ../catmux
		catmux_create_session projet.yaml
```

- Si on veut spécifier le nombre de robots (remplacer ? par le nombre de robots souhaités 2 ou 4)
```
		catmux_create_session --overwrite nb_robots=? projet.yaml
```

- Pour quitter la session:
```
		#Soit faire dans un terminal
		tmux kill-session
		#Ou le raccourci (ctrl+b into shift+k into y)
```

