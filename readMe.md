# robInfLib-matlab
```
Robot Inference Library (robInfLib) provides various demos of using Kernelized Movement Primitives, 
which is maintained by Dr. Yanlong Huang (University of Leeds)
Part of the codes are provided by Dr. João Silvério, Dr. Fares J. Abu-Dakka and Dr. Sylvain Calinon,
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



> <b>ORIENTATION KERNELIZED MOVEMENT PRIMITIVES <i>(demo_KMP_orientation.m, ref. [3])</i> </b>
<p align="center">
  **Demonstrated quaternions and angular velocities** <br>
  <img width="550" height="180" src="https://github.com/yanlongtu/robInfLib/blob/master/images/kmp_orientation_demos.png">
</p>
<p align="center">
  **Adaptation by using orientation-KMP** <br>
  <img width="550" height="180" src="https://github.com/yanlongtu/robInfLib/blob/master/images/kmp_orientation_ada.png">
</p>

> <b>UNCERTAINTY-AWARE KERNELIZED MOVEMENT PRIMITIVES <i>(demo_KMP_uncertainty.m, ref. [4])</i> </b>
<p align="center">
  **Covariance and uncertainty prediction by using KMP while considering adaptations** <br>
  <img width="800" height="180" src="https://github.com/yanlongtu/robInfLib/blob/master/images/kmp_uncertainty.png">
</p>

### REFERENCE

#### [1] KMP: learning and adaptation 
[[Link to publication]](https://www.researchgate.net/publication/331481661_Non-parametric_Imitation_Learning_of_Robot_Motor_Skills)
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
[[Link to publication]](https://www.researchgate.net/publication/319349682_Kernelized_Movement_Primitives)
[[Link to video]](https://www.youtube.com/watch?v=sepb6Vs3OMI&feature=youtu.be)
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
[[Link to publication]](https://www.researchgate.net/publication/330675655_Generalized_Orientation_Learning_in_Robot_Task_Space)
[[Link to video]](https://www.youtube.com/watch?v=swYJZfAWTHk&feature=youtu.be)
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

#### [4] Uncertainty-aware KMP: uncertainty/covariance prediction and adaptation
[[Link to publication]](https://www.researchgate.net/publication/334884378_Uncertainty-Aware_Imitation_Learning_using_Kernelized_Movement_Primitives)
[[Link to video]](https://www.youtube.com/watch?v=HVk2goCQiaA&feature=youtu.be)
```
@InProceedings{silverio2019uncertainty,
  Title = {Uncertainty-Aware Imitation Learning using Kernelized Movement Primitives},
  Author = {Silv\'erio, J. and Huang, Y. and Abu-Dakka, Fares J and Rozo, L. and  Caldwell, D. G.},
  Booktitle = {Proc. {IEEE/RSJ} International Conference on Intelligent Robots and Systems ({IROS})},
  Year = {2019},
  Pages={90--97}
}
```



