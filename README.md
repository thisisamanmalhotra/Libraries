# Libraries
armMasala.h is a single library which is capable of controlling the whole arm with just a couple of functions.

## HOW TO USE THE LIBRARY?

**Step 1: Declare the object of the class armMasala**

```c++
	armMasala arm
```

**Step 2: Declare the array of pins for each motor**

```c++
    int linearActuator1[4]={45,43,8,A3};  //pin1,pin2,pwm,pot
    int linearActuator2[4]={31,33,11,A5};
    
    int turntable[4]={1,127,1,A2}; //lower serial value,upper serial value ,serialno,pot pin
    int gripper[4]={129,256,2,0};
```

**NOTICE:** for turntable and gripper, argument are bit different as it uses sabertooth.


**Step 3: Booting...** 

Boot the pins. i.e., declare them as input or output.

```c++
arm.boot(linearActuator1,linearActuator2,turntable,gripper);
```

**Step 4: Ready to go...**

To get the normal motion of the actuators.

```c++
arm.LA1.velocity=50;
arm.LA1.run(0,1);  0 or 1 just like how hercules operates..
arm.TT.run(1,0);
```

**Step 5: For inverse motion** 

Just specify the direction ratio for each direction which ranges from -1 to +1.

```c++
    arm.gotoSemiDir(dr_in_x,dr_in_y,dr_for_TT,50);
```    

## Explanation

Library starts with a basic definition of class motor.

As the name suggests, object of this class represent each single motor and provides functionality for the same.
________________________________________________________________________
Data members include variables such as:
```
1:  pin[4] = {pin1_of_hercules,pin2_of_hercules,pwm,potPin} "Used for dec" 
2:  potval   "Stores the value of potentiometer"
3:  velocity "Stores the pwm value"
```

Member functions are:
```
1:  setPins(int pin[]) "initialises all input and output pins for motor"
2:  run(int p1,int p2) "runs the motor with hercules at pwm=velocity"
4:  getPotVal()        "Fetches the pot value using analogRead()"
```          
________________________________________________________________________


Second class is saberMotor

its data members are :
```
1: saberParameters[4]    "lower serialVaule ,upperSerialValue ,Serial no ,pot"
2: potVal
3: ifRunningForFirstTime "holds value 1 if ruuning first time"
4: acc                   "acceleration up ranges from (0,1) "
5: current               "represents the current state"
6: velocity              "represents the maximum velocity"
7: currentSpeed          "holds the value of current speed "
```
Member functions are:
```
1:  setPins(int temp[])
2:  run(1,0)
3:  getPotVal()
```
________________________________________________________________________
The next class is armMasala, which actually binds up all the objects of the previous class motor and have members as:
```
1: motor LA1 "for bottom linear actuator"
2: motor LA2 "for upper linear actuatior"
3: saberMotor TT  "For turn Table"
4: saberGripper GRP
```

Other than these there are some other data members as well which holds the physical dimentions of the arm those are:
```
4:  L[10] "Holds the value of link lengths"
5:  TP[6] "Dimentions of the triangle in the arm"
6:  TL[2] "Angle limits for the triangle"
7:  TTP[2]"corresponding pot values for the minimum and maximum angle"
8:  AL[4] "minimum and maximum lenght of the actuators"
9:  AP[4] "corresponding pot limits for the actuators"
```
Member functions are:
```
1: boot(int[],int[],int[],int[]) "Calls setpins for each motor taking pin[4]= {pin1_of_hercules,pin2_of_hercules,pwm,potPin} as arguments for each motor and for sabertooth"

2: gotoPot(double,double,double,double) "first three arguments are the final pot values of the LA1,LA2,TT which are required to be achieved and last ar value
    is the maximum pwm value which should not be exceeded during the motion"
	This function just simply calculate the difference to the final and initial pot value and assigns the velocity to motors based of the magnitude of the distance that they have to cover.

3: gotoSemiDir(double,double,double,double) "this function takes direction ratios for a particular direction as input which can range from -1 to +1 and the  
	last input is the pwm value 
	This function is based of the dynamic inverse kinametics algorithm which is responsible of dynamic assignment of pwm values to the actuators based on their current position (PresentPotValue) and the direction ratios that are provided as an input to achieve a given motion."
```
## HOW IT WORKS???????
Both the actuators are assumed to be in a plane and all the calculations are applied on that plane.

**Step 1:**	Just like to answer a question we need to know the question first, in the same way to achieve a motion in a plane we need to know the component of velocity of the endeffector in that plane i.e, dX/dt and dY/dt.
    
    So our first step is to calculate dX/dt and dY/dt as velocity in x and y direction is directionRatio*maximum_velocity_in_any_direction
        So dX/dt = direction_Ratio_for_xAxis * maximumPwm
        and dY/dt = directoin_Ratio_for_yAxis * maximumPwm

**Step 2:** Second Step is to calculate the present lenght of the actuators i.e, ALPHA(for LA1) and BETA(for LA2) which can be easily calculated using simple map function which maps pot values to the length

**Step 3:** Third step is to calculate present angle between the armBars ie THETA and FI which requires ALPHA and BETA as input.

**Step 4:** Fourth step is to calculate the given 2-D coordinates of the endeffector of the arm i.e, X and Y 

**Step 5:** Fifth step is to calculate the rate of change of angle FI and THETA i.e, dFI/dt and dTHETA/dt using dX/dt and dY/dt which was calculated in step 1.

**Step 6:** Step six is to calculate the rate of change of length of each actuator i.e, dALPHA/dt and dBETA/dt for the present position using dFI/dt and dALPHA/dt calculated in previous step.

**Step 7:** Step seven is to map the values of dALPHA/dt and dBETA/dt to the pwm values and get the direction of motion of each i.e, forward of backward based on the sign of dALPHA/dt and dBETA/dt.

All these steps are repeated as the loop runs and hence pwm and direction for each actuator is assigned each time the loop runs thus giving the required motion which was described by the direction ratios which were given as the argument to the function.

For turnTable there are no calculations other than simple mapping of the velocity to the value that is to be written to serial.

## Things that should be taken care of...

Remember to test whether the pot values increases if *arm.LA1.run(1,0)* is called.
This condition is the most important and must be checked before setting for competetion. 
