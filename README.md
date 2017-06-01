# simVSSS and Reinforcement learning

This project is a branch of the original simVSSS. The objective was develop tests in a primitive tabular and a end-to-end version of Deep Q-Learning (DQN - [Deepmind blog post](https://deepmind.com/research/dqn/ "Deep Q-Learning")) using
the simVSSS platform.

## Deep Q-Learning

The DQN algorithm is an attempt to achieve the goal of beat high-human levens in play atari games. The principle of the algorithm is that you 
enter the raw of pixels present in screen to a neural network, and its the backprop is made by correcting rewards in each taken action. The algorithm
can be represented in the figure below.

![DQN](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/nature_dqn.jpg "DQN")

The results achieved by the algorithm are very impressive, and the rank comparing both human level and DQN precision are the follows:

![DQN-rank](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/rank_dqn_nature.jpg "DQN-rank")

## simVSSS and DQN

The original idea was implement original algorithm, using the pixels, to train a neural net in a tabular problem. But the PC used in this tests have an AMD Graphics Card. Beyond it, the original problem took arround 38 days to train, which is very difficult to achieve even with a simpler problem. 
So, the idea here is use the cpu library for deep learning tiny-dnn. There are two versions of the algorithms in this repository: CNN and MLP. Both uses tabular
method, but for the reasons given above only the MLP will achive good results in short time (arround 2 hours). The implemented idea is shown below.

| Algorithm        | Representation   |
| ------------- |:-------------:| 
| CNN      | ![DQN-CNN](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/dqn-cnn.png "DQN-CNN") |
| MLP      | ![DQN-MLP](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/dqn-mlp.png "DQN-MLP")      |

## Results

The results with the MLP algoritm was quite reasonable. But it is dificult to see this algorihms implemented for real games in short future.
The loss show us that the problem converges.

[DQN-COST](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/cost.png "DQN-COST")

And the path plotted by the agent confirms the prediction.

[DQN-MAP](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/init_map.png "DQN-MAP")
[DQN-FINAL](https://github.com/lucasbsimao/DQN-simVSSS/tree/master/images/final_map.png "DQN-FINAL")