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