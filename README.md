# Predictor

Students :
Promo ROB 2019

In a context of industrial AUV (Autonomous Underwater Vehicule) applications, ingineers need to predict, before the mission, that their robot will not be in a forbidden area or lost.

This project aim to propose several methods able to guarantee margins of position error for submarin application (AUV). 

Several solutions have been deveploped. Some aim to quantify the AUV position uncertainty in order to define a maximum errorat the end of the mission. Others show that the robot is able to go through a cycle to infinity without ever getting lost. Finally a method is proposed to determine a navigation zone which we are certain that the robot never goes out.

## Kalman
A Kalman filter is used in prediction mode in order to quantify uncertainty on the AUV position during its mission. We are therefore able to quantify how much the robot is lost at the end of its mission.

## Méthode particulaire
This method completes and validates the work done by Kalman. It involves simulating the movement of a multitude of robots that all have different evolution and behavior. For the same mission, each robot will follow a different trajectory. At the end of the simulation we are able to quantify the uncertainty on the location but also to detect unexpected behaviors.

## Focalisation
In this part we try to determine a navigation zone from which we are certain that the robot never goes out. This ensures that the robot will not be lost during its mission and can always be recovered.

## Tubex
Tool used (or not) by the other groups. Examples of use can be found in this directory.

## Lien utiles

Trello: 

Latex/ Overleaf : https://www.overleaf.com/8358965573yyksgdngzxmz

Predictor web site: https://www.ensta-bretagne.fr/jaulin/predictor.html
