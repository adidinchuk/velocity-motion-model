# velocity-motion-model

Object for modeling motion - based on section 5.3 of the Probabilistic Robotics text.

## Dependencies
[numpy](https://pypi.org/project/numpy/)

[matplotlib](https://pypi.org/project/matplotlib/) - only used for visualization in the main method

## Demo

A main method is provided with a basic demo.


```bash
python .\velocity_motio_model.py
``` 
```bash
Executing __main__ method for the VMM class
Using alpha values: 0.001 0.001 0.001 0.001 0.001 0.001
Using initial pose: [1.         0.         1.57079633]
Particles used: 10
Movement steps: 150
Using command : 3.141592653589793 0.031415926535897934
```

## Usage

```python
from velocity_motio_model import VelocityMotionModel
import numpy as np

#initialize with uncertainty values
vmm = VelocityMotionModel(np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001]))  

#select and initial pose ([x, y, theta]), a move command ([v, w]) and the number of particles to generate
initial_pose = np.array([[1,0,np.pi/2]])
move_command = np.array([[1, np.pi/2]])
particles = 100

#generate a new set of particles
step_particles = vmm.sample_motion_model_velocity(initial_pose, move_command, particles, d_time=1) 

#update particle positions with a new move command (the same move command is used in this example)
#note that n=1 is used to generate exactly 1 new updated particle for each particle in the previous step
step_particles = vmm.sample_motion_model_velocity(step_particle, command, n=1, d_time=1)
```


## Sample Results

**Results of using a constant mote command of v=pi and w=pi/80**
```bash
Executing __main__ method for the VMM class
Using alpha values: 0.001 0.001 0.001 0.001 0.001 0.001
Using initial pose: [1.         0.         1.57079633]
Samples used: 100
Movement steps: 150
Using command : 3.141592653589793 0.039269908169872414
```
![alt text](https://github.com/adidinchuk/velocity-motion-model/blob/main/Figure_1.png?raw=true)

**Results of using a constant mote command of v=1 and w=0**
```bash
Executing __main__ method for the VMM class
Using alpha values: 0.001 0.001 0.001 0.001 0.001 0.001
Using initial pose: [1.         0.         1.57079633]
Samples used: 100
Movement steps: 150
Using command : 3.141592653589793 0.039269908169872414
```
![alt text](https://github.com/adidinchuk/velocity-motion-model/blob/main/Figure_2.png?raw=true)
**Results of using a constant mote command of v=pi and w=pi/400**
```bash
Executing __main__ method for the VMM class
Using alpha values: 0.001 0.001 0.001 0.001 0.001 0.001
Using initial pose: [1.         0.         1.57079633]
Samples used: 100
Movement steps: 150
Using command : 3.141592653589793 0.007853981633974483
```
![alt text](https://github.com/adidinchuk/velocity-motion-model/blob/main/Figure_3.png?raw=true)
