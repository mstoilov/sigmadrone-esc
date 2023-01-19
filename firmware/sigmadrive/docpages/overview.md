# Overview
@tableofcontents

Servo controller overview. TBD

## Introduction
FOC applies to three phase Brushless Motors that are operating in sinusoidal mode. 
In sinusoidal commutation, all three wires are permanently energized with a voltage
producing sinusoidal current that is 120 degrees apart on each phase. The current through
each phase produces manetic field which is perpendicular to the phase winding. The 3 phase
windings are situated 120 degrees apart. Flowing sinusoidal current through the phases produces
combined North/South magnetic field that rotates inside the motor cage. If a magnet is placed 
on a rotor inside this cage, its North and South poles will be pulled towards the South and 
North poles of the rotating field. Maximum torque is achieved when the rotor’s magnetic field 
is 90 degree apart from the stator field. If the magnet is aligned with the stator field it will
experience strong outwards pull but no torque or rotation. All other alignmets will produce some
torque, but to make the motor turn optimally we need to keep the rotor magnet at 90 degrees to 
the stator magnetic field. This means we need to know the position of the rotor in real time, 
then apply voltage on the U V W wires so that the magnetic field produced on the stator 
is 90 degrees apart.

![](foc.png)

### Block diagram of the torque closed loop

![](foc-block.png)

### Diagram steps:
1. Forward Clark Transformation \n
Convert the 3 phase current vectors: (I<sub>a</sbu>, I<sub>b</sub>, I<sub>c</sub>) to 
2 phase vectors (I<sub>&alpha;</sub>, I<sub>&beta;</sub>)\n 
I<sub>&alpha;</sub> = I<sub>a</sub> * 3 / 2 \n 
I<sub>&beta;</sub> = I<sub>b</sub> * &radic;3 / 2 - Ic * &radic;3 / 2 \n


2. Forward Park Transformation \n
Jump on the rotatating reference frame\n
Convert the 2 phase current vector: I<sub>&alpha;&beta;</sub> to vector I<sub>dq</sub> (in DQ-rotating reference frame)\n 
The D axis is aligned with the rotor flux. \n
I<sub>d</sub> = I<sub>&alpha;</sub> cos(&theta;) + I<sub>&beta;</sub> sin(&theta;) \n
I<sub>q</sub> = -I<sub>&alpha;</sub> sin(&theta;) + I<sub>&beta;</sub> cos(&theta;) \n


3. Reverse Park Transformation \n
V<sub>&alpha;</sub> = V<sub>d</sub> cos(&theta;) - V<sub>q</sub> sin(&theta;) \n
V<sub>&beta;</sub> = -V<sub>d</sub> sin(&theta;) + V<sub>q</sub> cos(&theta;) \n


4. Reverse Clark Transformation \n
V<sub>a</sub> = V<sub>&alpha;</sub> * 2 / 3 \n
V<sub>b</sub> = -V<sub>&alpha;</sub> / 3 + V<sub>&beta;</sub> / &radic;3 \n
V<sub>c</sub> = -V<sub>&alpha;</sub> / 3 - V<sub>&beta;</sub> / &radic;3 \n


## Design and important classes

### MotorDrive

The MotorDrive class is responsible for gathering all the data
involved in the mathematical calculations of the field oriented
control (FOC). The data includes:
* Vbus - The main voltage applied to the motor windings
* Current - The current going through the motor windins. By knowing the current through the motor phases
we can calculate the magnitude and the orientation of the magnetic field.
* Rotor position - By knowing the rotor position, and given the fact that the rotor magnetic field is caused
by permanent magnets we can know the orientation of that permanent magnetic field.

The MotorDrive class has functions like @ref MotorDrive::ApplyPhaseVoltage to set winding voltage. 

The MotorDrive class receives an Interrupt Service Request (ISR) at a fixed rate, normally between
20kHz and 48kHz. The ISR also signals that the ADC current reading has completed and the data is
ready to be received.

The MotorDrive class reads the encoder, but because the encoder might not be able to update the
rotor position as quickly as the rate of the ISR we need to read it with lesser rate.

### PwmGenerator
The PWM generator is the mechanism used by the MotorDrive to set variable voltage to the windings.
By controlling the PWM timings we effectively can control the voltage magnitude applied to the
different motor windings. Also, by controlling the voltage applied to the different windings we
controll the overall orientation of the magnetic field.