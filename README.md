# Tennis Ball Collector

Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique à l'ENSTA Bretagne en 2021.

## Lancer la simulation

### Dépendences

OpenCV

### Démarrer la simulation

Il faut tout d'abord lancer gazebo avec l'environnement qui ici est un terrain de tennis.
```
ros2 launch tennis_court tennis_court.launch.py
```
Ensuite il faudra faire apparaître le robot dans gazebo avec la commande suivante
```
ros2 launch scaraball_gazebo spawner.launch.py
```
La commande ci-dessous permet de lancer l'algorithme de détection des balles et des obstacles grâce à la caméra placée au dessus du terrain.
```
ros2 launch scaraball_camera position.launch.py
```
Enfin, on doit lancer l'algorithme de contrôle du robot avec la commande ci-dessous.
```
ros2 launch scaraball_control launch.launch.py
```

## Groupe
Groupe3

### Membres
###### Maha Halimi
###### Julien Piranda
###### Paul Pineau
###### Mourtaza Kassamaly
###### Quentin Vintras

### Gestion de projet

https://tree.taiga.io/project/mourtazakassamaly-tennisballcollector/timeline

## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.

### Package `scaraball_camera`

Le dossier `scaraball_camera` est un package ROS contenant le noeud qui permet de récupérer les positions du robot, des balles et des joueurs en utilisant le tracking d'OpenCV. Il permet de récupérer l'ordre d'apparition des balles.
Consulter le [README](scaraball_camera/README.md) du package pour plus d'informations.

### Package `scaraball_description`

Le dossier `scaraball_description` est un package ROS contenant les fichiers urdf de notre robot. Ce sont les fichiers qui permettent de créer la géométrie de notre robot et de l'afficher sous Gazebo.

### Package `scaraball_gazebo`

Le dossier `scaraball_gazebo` est un package ROS permettant de lancer le robot pour la visualisation dans Gazebo. Il contient uniquement 2 fichiers launch, un qui permet de lancer gazebo et avec le robot, et un qui permet de faire apparaitre le robot dans une simulation déjà existante. C'est ce deuxième fichier launch qui sera le plus utilisé car nous aurons déjà lancé l'environnement.

### Package `scaraball_control`

Le dossier `scaraball_control` est un package ROS contenant le noeud qui permet le contrôle du robot. Il contient plusieurs fichiers qui permettent la commande pour la navigation, mais aussi le choix des balles à aller récupérer et l'ouverture des pinces.
Consulter le [README](scaraball_control/README.md) du package pour plus d'informations.

### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)

### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
