# Scaraball_camera

Ce script récupère la position du robot, des balles et des humains présent sur le court de tennis. 
L'ordre d'apparition des balles est conservé. 
Les données sont renvoyées sur les topics /posHuman /posRob /posBall

## Lancement du package
On suppose que la simulation gazebo est déjà ouverte. 

```
ros2 launch scaraball_camera position.launch.py
```

