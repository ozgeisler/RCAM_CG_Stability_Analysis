
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

## Repository structure
