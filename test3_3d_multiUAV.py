#!/usr/bin/env python
"""Controller synthesis for system with continuous (linear) dynamics.

This example is an extension of `robot_discrete.py`,
by including continuous dynamics with disturbances.
The dynamics is linear over a bounded set that is a polytope.
"""
# Wenqi Han
# July 11, 2017 (last update)
# NO, system and cont. prop definitions based on TuLiP 1.x
# 2 Jul, 2013
# NO, TuLiP 1.x discretization
# 17 Jul, 2013


# @import_section@
import logging

import numpy as np
from tulip import spec, synth, hybrid
from polytope import box2poly
from tulip.abstract import prop2part, discretize
from tulip.abstract.plot import plot_partition
# @import_section_end@


logging.basicConfig(level=logging.WARNING)
show = False

# @dynamics_section@
# Problem parameters
input_bound = 1.0
uncertainty = .1

# Continuous state space
cont_state_space = box2poly([[0., 3.], [0., 3.],[0.,3.]])

# Continuous dynamics
A = np.array([[1.0, 0.,0.], [ 0., 1.0, 0.],[0., 0. ,1.0]])
B = np.array([[1.0, 0.,0.], [ 0., 1.0, 0.],[0., 0. ,1.0]])
E = np.array([[1,0,0], [0,1,0],[0,0,1]])

# Available control, possible disturbances
U = input_bound *np.array([[-1., 1.], [-1., 1.],[-1., 1.]])
W = uncertainty *np.array([[-1., 1.], [-1., 1.],[-1., 1.]])

# Convert to polyhedral representation
U = box2poly(U)
W = box2poly(W)

# Construct the LTI system describing the dynamics
sys_dyn = hybrid.LtiSysDyn(A, B, E, None, U, W, cont_state_space)
# @dynamics_section_end@

# @partition_section@
# Define atomic propositions for relevant regions of state space
cont_props = {}
cont_props['home'] = box2poly([[0., 1.], [0., 1.],[0.,1.]])
cont_props['lot'] = box2poly([[2.5, 3.], [2.5, 3.],[2.5,3.]])
cont_props['obs1']= box2poly([[0.,.8],[.4,1.2],[2.,2.5]])
cont_props['obs2']= box2poly([[.8,1.6],[.4,1.2],[2.,2.5]])
cont_props['obs3']= box2poly([[2.,2.6],[1.2,1.8],[1.9,2.5]])
# Compute the proposition preserving partition of the continuous state space
cont_partition = prop2part(cont_state_space, cont_props)
plot_partition(cont_partition) if show else None
# @partition_section_end@

# @discretize_section@
# Given dynamics & proposition-preserving partition, find feasible transitions
disc_dynamics = discretize(
    cont_partition, sys_dyn, closed_loop=True,
    N=8, min_cell_volume=0.1, plotit=show
)
# @discretize_section_end@

# Visualize transitions in continuous domain (optional)
plot_partition(disc_dynamics.ppp, disc_dynamics.ts,
               disc_dynamics.ppp2ts) if show else None

# Specifications
# Environment variables and assumptions
#env_vars = {'obsAct1'}
env_vars = {'obsAct1','obsAct2','obsAct3'}
env_init = {'obsAct1','!obsAct2','!obsAct3'}                # empty set
#env_prog = {'!obsAct1'}
env_prog = {'!obsAct1 || !obsAct2 || !obsAct3'}
env_safe = set()                # empty set

# System variables and requirements
sys_vars = {'uav1','uav2'}
sys_init = {'uav1','uav2'}
sys_prog = {'home'}               # []<>home
#sys_safe = {'(X(X0reach) <-> lot) || (X0reach && !park)'}
sys_safe ={'(X(uav1)) <-> (!obs1) || ((uav1) && !obsAct1)'}
sys_safe ={'(X(uav1)) <-> (!obs2) || ((uav1) && !obsAct2)'}
sys_safe ={'(X(uav1)) <-> (!obs3) || ((uav1) && !obsAct3)'}

sys_safe ={'(X(uav2)) <-> (!obs1) || ((uav2) && !obsAct1)'}
sys_safe ={'(X(uav2)) <-> (!obs2) || ((uav2) && !obsAct2)'}
sys_safe ={'(X(uav2)) <-> (!obs3) || ((uav2) && !obsAct3)'}

#sys_safe = {'(obsAct1) -> (!obs1)'}
#sys_safe |= {'(obsAct2 -> (!obs2)'}
sys_prog |= {'lot','uav1','uav2'}

# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
specs.qinit = '\E \A'

# @synthesize_section@
# Synthesize
ctrl = synth.synthesize('omega', specs,
                        sys=disc_dynamics.ts, ignore_sys_init=True)
assert ctrl is not None, 'unrealizable'


# Generate a graphical representation of the controller for viewing
#if not ctrl.save('continuous_test3.png'):
print(ctrl)
# @synthesize_section_end@

# Simulation
