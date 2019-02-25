####Partie Python:

## Installation 

Pour pouvoir utiliser la partie Python et son affichage, nous utilisons une application de Rémi RIGAL et Noélie RAMUZAT pour faire le lien entre Unity3D et Python.

https://github.com/RemiRigal/Unity-Vibes/

https://github.com/NoelieRamuzat/PyUnityVibes

Il faut installer une librairie et télécharger une application:

Téléchargement de l'application selon votre OS:
    *https://github.com/RemiRigal/Unity-Vibes/release*

```
# Instalation de PyUnityVibes 
pip3 install PyUnityVibes
```

Ensuite, pour ceux qui sont sous Linux (je n'ai pas encore eu l'occasion de travailler sous Windows). Un makefile a été fait pour que cela fonctionne sous Linux.

```
cd Python/
make run
```

Ensuite, le programme calcul l'avancement 
```
>>>> Run launched ...
Submarine  1 created
Submarine  2 created
Submarine  3 created
Submarine  4 created
Submarine  5 created
Submarine  6 created
Submarine  7 created
Submarine  8 created
Submarine  9 created
Submarine  10 created
Ajout des objets à l'animation
t = 130.000000 
```
Ensuite, à la fin des 130s, le programme affiche *Done !* puis lance l'affichage sur la fenêtre UnityVibes.

