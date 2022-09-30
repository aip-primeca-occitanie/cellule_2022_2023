# Fonctionnement du package ROS
### config
Ces fichiers sont chargés au lancement de RViz.

Le fichier .srdf contient des groupes (DNP1, ..) dont la suite de points permet à Moveit de construire une trajectoire de pick and place 
et correspond à la fonction DeplacerPiece() de CoppeliaSim.

### launch
Permet de lancer l'environnement RViz/Moveit avec le robot avec la commande :
```bash
roslaunch staubli_tx2_60l_moveit demo.launch
```

Si on veut se connecter au vrai robot :
```bash
```
### meshes
Contient les fichiers de modélisation 3D du robot et de son environnement. Elles sont utilisées dans les fichiers urdf.

### scripts
Le script permet de construire une trajectoire à partir des points définis dans le fichier .srdf et la trajectoire se déclenche avec la commande DeplacerPiece()
de CoppeliaSim.

### urdf
tx2_60l.xacro est le fichier loader par les launchs. Il contient l'ensemble des urdf et définit leurs liens dans le monde virtuel.


