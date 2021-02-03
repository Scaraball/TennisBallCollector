# Tennis Ball Collector

Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique à l'ENSTA Bretagne en 2021.


## Lancer la simulation

### Dépendences


OpenCV


### Démarrer la simulation
```
ros2 launch my_robot_gazebo spawner.launch.py
```


## Groupe
Groupe3

### Membres
Maha Halimi
Julien Piranda
Paul Pineau
Mourtaza Kassamaly
Quentin Vintras


### Gestion de projet

https://tree.taiga.io/project/mourtazakassamaly-tennisballcollector/timeline



## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
