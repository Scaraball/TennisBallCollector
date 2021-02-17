# Objectifs du 17/02/2021

PO: Quentin Vintras


## Fonctionnalités attendues
#### 1- Tester les algorithmes précédemment développés et regarder si toute la chaîne fonctionne sans le nouvelles contraintes
#### 2- Eviter les joueurs sur le terrain

## Tâches à réaliser

#### 1- Récupérer la position des joueurs 
#### 2- Coder un algorithme de navigation permettant l'évitement des joueurs (méthode des potentiels artificiels)

## Challenges techniques

#### La distorsion de l'image ainsi que la couleur assez neutre des joueurs rend difficile la segmentation pour la récupération de la position des obstacles

#### Les joueurs se déplacent rapidement sur le cours et sont susceptibles de renverser le robot en cas de collision. Il faut donc veiller à ne pas heurter le joueur mais aussi à ce que le joueur ne nous heurte pas.

#### Il va falloir recalculer le champ de potentiels à chaque fois que la position des obstacles évolue.

