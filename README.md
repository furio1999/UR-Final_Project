# UR-Final_Project

Project for [Underactuated Robots Course](http://www.diag.uniroma1.it/~oriolo/ur/) at Sapienza University of Rome\
\
<a href="https://www.dis.uniroma1.it/"><img src="http://www.dis.uniroma1.it/sites/default/files/marchio%20logo%20eng%20jpg.jpg" width="500"></a>

## Instructions
```
# clone
git clone https://github.com/furio1999/UR-Final_Project.git
```

```
# push changes
git push
# if it does not work, do this
git push -uf origin main
```

```
# update your local folder
git pull
```

### Modeling
```
Compute_Model.m
```
### MPC
```
Main_MPC.m
```
### Neural Network
```
Neural_network.m
```
### MPC warm-start
```
Main_warm.m
```
### Tracking
```
Main_Tracking.m
```


## Modeling
- Lagrangian Formulation for 2D quadruped robot
- Developed from scratch

## Control
- Model Predictive Controller
- Training of Feedforward Neural Network to generate trajectories from initial state x(0)
- NN-boosted MPC control

## Simulation
```
Draw_robot.m
```

https://user-images.githubusercontent.com/63920397/162575745-fa89b125-d6ce-468a-9323-7e8ee53cdeb0.mp4



## Contributors
- **Saverio Borrelli** <a href="https://www.linkedin.com/in/saverioborrelli/"><img src="https://www.tecnomagazine.it/tech/wp-content/uploads/2013/05/linkedin-aggiungere-immagini.png" width="30"></a>
- **Monica De Pucchio** <a href="https://www.linkedin.com/in/monica-de-pucchio/"><img src="https://www.tecnomagazine.it/tech/wp-content/uploads/2013/05/linkedin-aggiungere-immagini.png" width="30"></a>
- **Fulvio Sanguigni** <a href="https://www.linkedin.com/in/furio19/"><img src="https://www.tecnomagazine.it/tech/wp-content/uploads/2013/05/linkedin-aggiungere-immagini.png" width="30"></a>
