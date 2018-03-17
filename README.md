# Introduction
This repository provides minimal python implementation of an analytical forward/inverse kinematics for industrial 6-axis manipulator.
The implementation is based on the paper, "An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist", authored by Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur.

# Notes
The code is under development. The code does not deal with singular configuration explicitly. Moreover, configuration selection functionality is not completed, that is, all possible inverse kinematics solutions are returned simultaneously.

# TODO
-Implement singular configuration checker
-Configuration selection functionality
-Configuration plotting functionality (for debugging use)
