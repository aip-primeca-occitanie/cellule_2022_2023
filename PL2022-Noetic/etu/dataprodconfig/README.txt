
Nomenclature des noms des fichiers de données : dataX_Y_Z_nT.proconfig.h
----------------------------------------------

Où

X = 2 si tous les produits ne nécessitent que les postes 1, 2, 3, 4 (et donc les 2 premiers robots)
    4 sinon


Y = nombre de types de produits différents fabriqués (valeur allant de 1 à 6)

Z = nombre maximum d'unités pour chaque type de produit (si chaque type de produit a un nombre d'unités différent, choisir la valeur max)

T = numéro de référence de l'instance parmi celles de type X_Y_Z (0 pour la première instance de type X_Y_Z, 1 pour la deuxième, ...)




Attention ! Pour le moment, le checker et certains éléments semblent bloqués à 3 opérations max par produit, donc pour l'instant éviter des produits à plus de 3 tâches