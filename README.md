
# RCAM Longitudinal Stability Analysis

MATLAB implementation of trim, linearization, and longitudinal stability analysis for the **Research Civil Aircraft Model (RCAM)** benchmark, focused on how center of gravity (CG) location affects static and dynamic longitudinal stability.

## Overview

This project answers: how does moving the CG fore/aft change the aircraft's pitching moment behavior, its neutral point, its trim condition in descending flight, and the eigenvalues of its linearized longitudinal dynamics?

The analysis is done in five stages:

1. **Wind-tunnel-style pitching moment sweep** — compute `Cm_cg''` (pitching moment coefficient about the CG, wind axis) vs. angle of attack, for CG locations ranging from `-0.3*cbar` to `0.5*cbar`, at fixed trim control deflections.
2. **Neutral point estimate** — read the CG location where `∂Cm_cg''/∂α` changes sign, off the plot from stage 1.
3. **Trim sweep** — trim the aircraft (Va = 67.6 m/s, γ = -3°, steady descent) at each CG location using `fminsearch` on an implicit trim residual.
4. **Linearization sweep** — linearize the nonlinear 6DOF model about each trim point using implicit numerical differencing, producing one `A`, `B` pair per CG location.
5. **Eigenvalue analysis** — extract the longitudinal 4-state subsystem (`u, w, q, theta`) from each linear model, plot eigenvalues across CG locations, and compare against the neutral point estimate from stage 2.

## Requirements

- MATLAB (no toolboxes required — uses base `fminsearch`, `eig`, `svd`)

## How to run

1. Run `trim_straight_level.m` once to generate `trim_values_straight_level.mat` (used as a fallback seed).
2. Run `part3a_Cm_vs_alpha.m` to reproduce the pitching-moment-vs-alpha plot and estimate the neutral point.
3. Run `trim_sweep_descend_vs_Xcg.m` to trim the descending condition at every CG location and save `trim_values_descend_vs_Xcg.mat`.
4. Run `part3d_linearize_vs_Xcg.m` to linearize about each trim point and save `linear_models_vs_Xcg.mat`.
5. Run `part3e_eigenvalues_vs_Xcg.m` to plot the longitudinal eigenvalues and print the stability table.

## Notes

- `fminsearch` on this 14-variable trim problem converges slowly on some CG locations due to poor variable scaling; trim scripts re-seed `fminsearch` across multiple passes per CG and report the residual cost `f0` so convergence can be checked directly rather than assumed from smooth-looking output.
- The pitching moment coefficient is computed purely from aerodynamics (no engine contribution), consistent with a wind-tunnel measurement, and evaluated at zero sideslip so the wind-axis and body-axis pitching moment coincide.

## Background

RCAM is a nonlinear 6-DOF rigid-body aircraft model with 9 states (`u, v, w, p, q, r, phi, theta, psi`) and 5 controls (aileron, stabilizer, rudder, and two throttle inputs), originally developed as a GARTEUR benchmark for flight control research.
