# Spacecraft Landing with MPC

This project pioneers the application of advanced model predictive control (MPC) techniques to enhance the precision and reliability of spacecraft landings. Utilizing MATLAB and Simulink, the initiative integrates sophisticated algorithms to maneuver spacecraft during the critical phases of powered descent and landing. In the *MATLAB* scripts **Code.m** we calculate the trajectories using **Model Predictive Control** saved in **Rocket.mat** and in **Simulation_3D.m** we simulates a 3D descent dynamic.

Central to the project is the development and implementation of a linear MPC framework, outlined in the Paper.pdf, which discusses the theoretical underpinnings and practical applications of thrust vectoring control for spacecraft. After some semplification the model used is expressed in the file **model.md**.




# **Model Predictive Control for Powered Descent Guidance and Control**

In this project, we explore the real-time implementation of Model Predictive Control (MPC) for spacecraft guidance, with a focus on thrust vector control during ascent and descent.

## The Model

The vehicle in the study is modeled as a rigid body with six degrees of freedom (6-DoF). Forces and torques from aerodynamics, gravity, and the propulsion system act on the spacecraft's center of gravity (CoG), influencing its motion.

The study introduces inertial and body reference frames (depicted in Figure 1), where the former is fixed in space, and the latter is moving, linked to the rocket.

![reference frames](Images/model.png)

It is possible to define the equations of motion (EoM) for the translational and rotational dynamics by means of Newton’s second law:

![equations of motion](Images/equations.png)

where

\[
\begin{align*}
\begin{bmatrix}
\ddot{x} \\
\ddot{y} \\
\ddot{z}
\end{bmatrix}^\top & \text{ are the body’s linear accelerations along } X_I, Y_I, Z_I \text{ (inertial frame).} \\
\begin{bmatrix}
\mathbf{F}_{bx} \\
\mathbf{F}_{by} \\
\mathbf{F}_{bz}
\end{bmatrix}^\top & \text{ are the forces applied to the body.} \\
\begin{bmatrix}
\mathbf{g}_{x} \\
\mathbf{g}_{y} \\
\mathbf{g}_{z}
\end{bmatrix}^\top & \text{ is the gravity force.} \\
\begin{bmatrix}
\ddot{\phi} \\
\ddot{\theta} \\
\ddot{\psi}
\end{bmatrix}^\top & \text{ are the angular accelerations.} \\
\begin{bmatrix}
\mathbf{M}_{x} \\
\mathbf{M}_{y} \\
\mathbf{M}_{z}
\end{bmatrix}^\top & \text{ are the moments about the Center of Gravity.} \\
\begin{bmatrix}
\mathbf{I}_{xx} \\
\mathbf{I}_{yy} \\
\mathbf{I}_{zz}
\end{bmatrix}^\top & \text{ are the inertia terms.} \\
\end{align*}
\]

### Creating the Model

To use the MPC method on this system, we need additional equations to treat this second-order ODE as a first-order expression. We reformulate the second-order ODEs as first-order ODEs by introducing the following states:

\[
\begin{align*}
x_1 &= x \\
x_2 &= y \\
x_3 &= z \\
x_4 &= \phi \\
x_5 &= \theta \\
x_6 &= \psi \\
x_7 &= \dot{x} \\
x_8 &= \dot{y} \\
x_9 &= \dot{z} \\
x_{10} &= \dot{\phi} \\
x_{11} &= \dot{\theta} \\
x_{12} &= \dot{\psi} \\
\end{align*}
\]

Now the model can be rewritten as a first-order ODE:

\[
\begin{cases}
\dot{x}_1 = x_7 \\
\dot{x}_2 = x_8 \\
\dot{x}_3 = x_9 \\
\dot{x}_4 = x_{10} \\
\dot{x}_5 = x_{11} \\
\dot{x}_6 = x_{12} \\
\dot{x}_7 = \frac{U_4}{m} \cdot \cos(x_5) \cdot \cos(x_6) - g \\
\dot{x}_8 = \frac{U_4}{m} \cdot \cos(x_5) \cdot \sin(x_6) \\
\dot{x}_9 = -\frac{U_4}{m} \cdot \sin(x_5) \\
\dot{x}_{10} = \frac{U_1}{I_{xx}} \\
\dot{x}_{11} = \frac{U_2}{I_{yy}} \\
\dot{x}_{12} = \frac{U_3}{I_{zz}} \\
\end{cases}
\]

### Conclusion

This reformulated model can now be used to apply the MPC method, facilitating the control of the spacecraft's trajectory and orientation during powered descent.





