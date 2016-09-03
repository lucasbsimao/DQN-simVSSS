# Deep Q-Learning

This repository has an implementation of Deep Q Learning based on Deep Minds results from Playing Atari with Deep Reinforcement Learning paper. The goal of this project is to make a maze where the agent have to achieve the ball.

![screenshot 1](https://raw.githubusercontent.com/Lucas001/Deep-Q-Learning/images/top.png)

Every time the robot reaches the ball, the game is reinitialized with the same map to simplify the learning.

The difference between Code-ANN and Code-CNN is that Code-ANN uses as state a matrix of 15x15 indicating the places where each object is placed on the map. In the other hand Code-CNN uses the pixels from window created by glut.

The code uses CPU only.

The only library required is *sudo apt-get install freeglut3-dev* 
