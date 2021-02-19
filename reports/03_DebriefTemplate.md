# Debrief du 17/02/2021

PO: Quentin Vintras


## Bilan

Pourcentage de tâches réalisées: 80 %

### Ce qui a fonctionné

Nous avons réussi à récupérer la position des obstacles, la navigation par champ de potentiels artificiels est assez bonne, nous avons des pinces fonctionnelles qui arrivent plutôt bien à récupérer les balles. 


### Ce qui n'a pas fonctionné

Nous avons eu du mal à naviguer correctement pour récupérer les balles, l'actualisation de la liste de balles à aller chercher ne se fait pas encore. Le temps passé à essayer d'optimiser la pince nous a fait prendre du retard sur l'optimisation du trajet même si une noeud a été développé.


### Retour d'expérience du PO

Il est assez difficile de coordonner les différentes composantes du groupe surtout lorsque les travaux n'avancent pas en même temps. Une meilleure gestion des ressources peut sans doute rectifier ce problème même si il est assez difficile de diviser des tâches qui nécessitent de travailler sur un même fichier. Aujourd'hui, le controleur était très dense et nous avons donc essayé de le séparer en deux parties, une partie qui envoie la position à laquelle doit se rendre le robot, et une partie qui permet la navigation vers cette position. Cela permet une meilleure lisibilité du code ainsi qu'une meilleure division du travail car cela permet à plusieurs membres du groupe de travailler sur une même user story en parallèle.


### Conseils pour le prochain PO

Il faudra bien penser à l'architecture du code et à comment on peut séparer certaines parties pour ne pas avoir de membres surchargés et d'autres inactifs. Penser à de nouvelles fonctionnalités à développer en amont peut aussi permettre de gagner du temps en codant des bouts pouvant servir plus tard.



## Nouvelles mesures

#### Les nouvelles mesures sont :
###### - Navigation par champ de potentiels artificiels
###### - Modification du nom des packages
###### - Séparation de la partie navigation et de la partie envoie des informations de navigation (un noeud envoie la position où le robot doit se rendre, un autre permet de faire rouler le robot jusqu'à cette postion.)
