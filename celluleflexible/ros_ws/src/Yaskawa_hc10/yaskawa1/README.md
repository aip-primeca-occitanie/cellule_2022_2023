# Fonctionnement du package ROS
### config
Ces fichiers sont chargés au lancement de RViz.

Le fichier .srdf contient des groupes (DNP1, ..) dont la suite de points permet à Moveit de construire une trajectoire de pick and place 
et correspond à la fonction DeplacerPiece() de CoppeliaSim.

### launch
Permet de lancer l'environnement RViz/Moveit avec le robot avec la commande :
```bash
roslaunch yaskawa1 demo.launch
```

Si on veut se connecter au vrai robot :
```bash
roslaunch yaskawa1 demo.launch sim:=false robot_ip:=192.168.1.40 controller:=yrc1000
```
### meshes
Contient les fichiers de modélisation 3D du robot et de son environnement. Elles sont utilisées dans les fichiers urdf.

### scripts
Le script permet de construire une trajectoire à partir des points définis dans le fichier .srdf et la trajectoire se déclenche avec la commande DeplacerPiece()
de CoppeliaSim.

### urdf
hc10_with_env.xacro est le fichier loader par les launchs. Il contient l'ensemble des urdf et définit leurs liens dans le monde virtuel.



