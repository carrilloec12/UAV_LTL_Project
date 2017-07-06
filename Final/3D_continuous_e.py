#!/usr/bin/env python
"""Controller synthesis for system with continuous (linear) dynamics.

This example is an extension of `robot_discrete.py`,
by including continuous dynamics with disturbances.
The dynamics is linear over a bounded set that is a polytope.
"""
# Petter Nilsson (pettni@kth.se)
# August 14, 2011
# NO, system and cont. prop definitions based on TuLiP 1.x
# 2 Jul, 2013
# NO, TuLiP 1.x discretization
# 17 Jul, 2013

# Note: This code is commented to allow components to be extracted into
# the tutorial that is part of the users manual.  Comments containing
# strings of the form @label@ are used for this purpose.

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
uncertainty = 0.0

# Continuous state space
cont_state_space = box2poly([[0., 3.], [0., 3.], [0., 3.]])

# Continuous dynamics - 3D
A = np.array([[1.0, 0., 0.], [ 0., 1.0, 0.], [0., 0., 1.0]])
B = np.array([[0.5, 0., 0.], [ 0., 0.5, 0.], [0., 0., 0.5]])
E = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])

# Available control, possible disturbances
U = input_bound *np.array([[-1., 1.], [-1., 1.], [-1., 1.]])
W = uncertainty *np.array([[-1., 1.], [-1., 1.], [-1., 1.]])

# Convert to polyhedral representation
U = box2poly(U)
W = box2poly(W)

# Construct the LTI system describing the dynamics
sys_dyn = hybrid.LtiSysDyn(A, B, E, None, U, W, cont_state_space)
# @dynamics_section_end@

# @partition_section@
# Define atomic propositions for relevant regions of state space
cont_props = {}
cont_props['home'] = box2poly([[0., 1.], [0., 1.], [0., 1.]])
cont_props['goal'] = box2poly([[2., 3.], [2., 3.], [2., 3.]])
cont_props['obsX1'] = box2poly([[0., 1.], [2., 3.], [0., 1.]]) 
cont_props['obsX2'] = box2poly([[1., 2.], [1., 2.], [1., 2.]])

# Compute the proposition preserving partition of the continuous state space
cont_partition = prop2part(cont_state_space, cont_props)
plot_partition(cont_partition) if show else None
# @partition_section_end@

# @discretize_section@
# Given dynamics & proposition-preserving partition, find feasible transitions
disc_dynamics = discretize(
    cont_partition, sys_dyn, closed_loop=True,
    N=8, min_cell_volume=1, plotit=show
)
# @discretize_section_end@

# Visualize transitions in continuous domain (optional)
plot_partition(disc_dynamics.ppp, disc_dynamics.ts,
               disc_dynamics.ppp2ts) if show else None

# # Environment variables and specification
# # @environ_section@
env_vars = {'obs1': 'boolean', 'count': range(2)}  
env_init = {'obs1', 'count = 0'}    
env_prog = '!obs1' 
env_safe = {'(!obs1 ->  (X count = count))', '(obs1 ->  (X count = count + 1))'}#, 'count<2'} 
# # @environ_section_end@

# # @specs_setup_section@
# # Augment the system description to make it GR(1)
# #! TODO: create a function to convert this type of spec automatically
sys_vars = {'UAV'}          # infer the rest from TS
sys_init = {'UAV'}
sys_prog = {'goal', 'home'}             # []<>goal
sys_safe = {'(obs1 -> (X !obsX1))||(UAV && !obs1)'} 
sys_prog |= {'UAV'}


# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
specs.qinit = '\E \A'

# @synthesize_section@
# Synthesize
ctrl = synth.synthesize('omega', specs,
                        sys=disc_dynamics.ts, ignore_sys_init=True)
assert ctrl is not None, 'unrealizable'



print disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(2)]

print disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(8)]

print disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(4)]

print disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(0)]
# Generate a graphical representation of the controller for viewing
if not ctrl.save('continuous.png'):
    print(ctrl)
# @synthesize_section_end@

# Simulation
