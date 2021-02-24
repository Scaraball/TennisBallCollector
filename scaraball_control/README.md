# Scaraball_control

Ce package récupère les positions des balles successives et envoie les points à atteindre successivement au robot. Il gère également les pauses et les temps de jeu.
Le déplacement du robot se fait par champ de potentiels artificiels.

## Lancement du package
En supposant que le world sur Gazebo est déjà lancé avec le robot dedans

```
ros2 launch scaraball_control launch.launch.py
```

