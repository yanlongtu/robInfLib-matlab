### robInfLib-matlab
```
This library provides various demos of using kmp.

These codes are mainly written by Dr. Yanlong Huang.

Part of the codes are provided by Dr. Sylvain Calinon, Dr. Fares J. Abu-Dakka and Dr. João Silvério,
which have been acknowledged in the corresponding files.
```

### DEMOS ILLUSTRATIONS

> <b>KERNELIZED MOVEMENT PRIMITIVES I <i>(demo_KMP01.m, ref. [1])</i> </b> 
<p align="center">
  **Model letters 'G' using GMM/GMR**<br>
  <img width="720" height="160"  src="https://github.com/yanlongtu/robInfLib/blob/master/images/modelLetterG.png">
</p>

<p align="center">
  **Trajectory adaptation by using KMP (mean)** <br>
  <img width="1400" height="150"  src="https://github.com/yanlongtu/robInfLib/blob/master/images/kmp_adaptationG.png">
</p>



> <b>KERNELIZED MOVEMENT PRIMITIVES II <i>(demo_KMP02.m, ref. [2])</i> </b> 
<p align="center">
  **Model letters 'B' using GMM/GMR**<br>
  <img width="720" height="160"  src="https://github.com/yanlongtu/robInfLib/blob/master/images/modelLetterB.png">
</p>

<p align="center">
  **Trajectory adaptation by using KMP (mean and covariance)** <br>
  <img width="720" height="180"  src="https://github.com/yanlongtu/robInfLib/blob/master/images/kmp_adaptationB.png">
</p>



> <b>ORIENTATION KERNELIZED MOVEMENT PRIMITIVES <i>(demo_orientationKMP.m, ref. [3])</i> </b>
<p align="center">
  **Demonstrated quaternions and angular velocities** <br>
  <img width="550" height="180" src="https://github.com/yanlongtu/robInfLib/blob/master/images/orientation_kmp_data.png">
</p>
<p align="center">
  **Adaptation by using orientation-KMP** <br>
  <img width="550" height="180" src="https://github.com/yanlongtu/robInfLib/blob/master/images/orientation_kmp_ada.png">
</p>



### REFERENCE

#### [1] KMP: learning and adaptation 
[Link to publication](https://www.researchgate.net/publication/331481661_Non-parametric_Imitation_Learning_of_Robot_Motor_Skills)
```
@InProceedings{Huang19ICRA_1,
   Title = {Non-parametric Imitation Learning of Robot Motor Skills},
   Author = {Huang, Y. and Rozo, L. and Silv\'erio, J. and Caldwell, D. G.},
   Booktitle = {Proc. {IEEE} International Conference on Robotics and Automation ({ICRA})},
   Year = {2019},
   Address = {Montreal, Canada},
   Month = {May},
   Pages = {5266--5272}
}
```

#### [2] KMP: learning, adaptation, superposition and extrapolaton. 
[Link to publication](https://www.researchgate.net/publication/319349682_Kernelized_Movement_Primitives)
```
@Article{Huang19IJRR,
  Title = {Kernelized Movement Primitives},
  Author = {Huang, Y. and Rozo, L. and Silv\'erio, J. and Caldwell, D. G.},
  Journal = {International Journal of Robotics Research},
  Year = {2019},
  Volume={38},
  Number={7},
  Pages = {833--852},
}
```


#### [3] KMP orientation: learning and adaptation
[Link to publication](https://www.researchgate.net/publication/330675655_Generalized_Orientation_Learning_in_Robot_Task_Space)
```
@InProceedings{Huang19ICRA_2,
   Title = {Generalized Orientation Learning in Robot Task Space},
   Author = {Huang, Y. and Abu-Dakka, F. and Silv\'erio, J. and Caldwell, D. G.},
   Booktitle = {Proc. {IEEE} International Conference on Robotics and Automation ({ICRA})},　　　　
   Year = {2019},
   Address = {Montreal, Canada},
   Month = {May},
   Pages = {2531--2537}
 }

```

