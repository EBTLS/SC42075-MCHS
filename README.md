# SC42075-MCHS

This repo contains the assignments from TU Delft SC42075 Modelling and Control of Hybrid System Course given by Professor. Bart De Schutter

**Lecturer**: Professor. Bart De Schutter

**Content:** This course mainly focus on analyze and controller design of Hybrid Systems

**Team Members**: Jiaxuan Zhang, Yiting Li

The assignments have two parts

# Part 1

We need to describe a real-life hybrid system not mentioned in course and model it by hybrid automata.

We choose the thermal management system of personal computers

# Part 2

We need to design a MPC controller for the car's adaptive cruise system following 10 steps. The 10 steps mainly contains:

1. Approximate nonlinear friction force with piecewise affine functions to achieve smallest approximation error
2. Transform approximated original model to a Mixed-Logical-Dynamics(MLD) Model
3. Design MPC controller for the MLD model without use direction function from toolbox, that is turn it into an optimization problem and call optimization solver
4. Use explicit MPC method to design explicit MPC controller and compare it to the original MPC controller

