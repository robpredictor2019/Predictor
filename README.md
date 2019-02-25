# Predictor

Étudiants :
Promo ROB 2019


Ce projet à pour but de proposer des méthodes capables de garantir des marges d'erreur de positionement pour des robots sous marins. Différentes solutions sont dévloppées. Certaines cherchent à quantifier l'incertitude sur la position du robot afin de définir une erreur maximal en fin de mission. D'autres montrent que le robot est capable de parcourir un cycle à l'infini sans jamais se perdre. Enfin une méthode est proposée afin de déterminer une zone de navigation de laquel on est certain que le robot ne sort jamais.
 
## Kalman
On utilise un filtre de Kalman en mode prédiction afin de quantifier l'incertitude sur la position du robot au cours de la mission. On est donc capable de quantifier de combien le robot c'est perdu à la fin de sa mission.

## Méthode particulaire
Cette méthode complète et valide le travail effectué par Kalman. Il s'agit de simuler le déplacement d'une multitude de robots qui ont tous une évolution et un comportement différents. Pour une même mission, chaque robot va donc suivre un trajectoire différente. A la fin de la simulation on est donc capable de quantifier l'incertitude sur la localisation mais aussi de détecter des comportements inatendus. 

## Focalisation
Dans cette partie on essaye de déterminer une zone de navigation de laquelle on est certain que le robot ne sort jamais. Cela permet de garantir que le robot ne se perdra pas lors de sa mission et qu'on pourra toujours le récupérer.

## Tubex
Outil utiliser (ou pas) par les autres groupes. On trouve dans ce repertoire.

##Lien utiles
