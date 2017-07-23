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
import tomatlab
import numpy as np
from tulip import spec, synth, hybrid
from polytope import box2poly
from tulip.abstract import prop2part, discretize, find_controller
from tulip.abstract.plot import plot_partition
# @import_section_end@
import random
visualize = False
from tulip.abstract.plot import plot_partition
logging.basicConfig(level=logging.WARNING)
show = False

# @dynamics_section@
# Problem parameters
input_bound = 5.0
uncertainty = 1.

# Continuous state space
cont_state_space = box2poly([[0., 50.], [0., 50.]])

# Continuous dynamics
A = np.array([[1.0, 0.], [ 0., 1.0]])
B = np.array([[2., 0.], [ 0., 2.]])
E = np.array([[1,0], [0,1]])

# Available control, possible disturbances
U = input_bound *np.array([[-1., 1.], [-1., 1.]])
W = uncertainty *np.array([[-1., 1.], [-1., 1.]])

# Convert to polyhedral representation
U = box2poly(U)
W = box2poly(W)

# Construct the LTI system describing the dynamics
sys_dyn = hybrid.LtiSysDyn(A, B, E, None, U, W, cont_state_space)
# @dynamics_section_end@

# @partition_section@
# Define atomic propositions for relevant regions of state space
cont_props = {}
cont_props['home'] = box2poly([[0., 10.], [0., 10.]])
cont_props['lot'] = box2poly([[45., 50.], [45., 50.]])
cont_props['obs1']= box2poly([[0.,8.],[4.,12.]])
cont_props['obs2']= box2poly([[8.,16.],[4.,12.]])
cont_props['obs3']= box2poly([[16.,24.],[12.,20.]])
# Compute the proposition preserving partition of the continuous state space
cont_partition = prop2part(cont_state_space, cont_props)
plot_partition(cont_partition) if show else None
# @partition_section_end@

# @discretize_section@
# Given dynamics & proposition-preserving partition, find feasible transitions
disc_params = {'closed_loop':True, 'N':8, 'min_cell_volume':0.1,
               'plotit':visualize, 'conservative':False}
disc_dynamics = discretize(cont_partition, sys_dyn, **disc_params)

# @discretize_section_end@

# Visualize transitions in continuous domain (optional)
plot_partition(disc_dynamics.ppp, disc_dynamics.ts,
               disc_dynamics.ppp2ts) if show else None

# Specifications
# Environment variables and assumptions
env_vars = {'obsAct1'}
#env_vars = {'obsAct1','obsAct2','obsAct3'}
env_init = {'obsAct1'}
#env_init = {'obsAct1','!obsAct2','!obsAct3'}                # empty set
env_prog = {'!obsAct1'}
#env_prog = {'!obsAct1 || !obsAct2 || !obsAct3'}
env_safe = set()                # empty set

# System variables and requirements
sys_vars = {'X0reach'}
sys_init = {'X0reach'}
sys_prog = {'home'}               # []<>home
#sys_safe = {'(X(X0reach) <-> lot) || (X0reach && !park)'}
sys_safe ={'(X(X0reach)) <-> (!obs1) || ((X0reach) && !obsAct1)'}
#sys_safe ={'(X(X0reach)) <-> (!obs2) || ((X0reach) && !obsAct2)'}
#sys_safe ={'(X(X0reach)) <-> (!obs3) || ((X0reach) && !obsAct3)'}

#sys_safe = {'(obsAct1) -> (!obs1)'}
#sys_safe |= {'(obsAct2 -> (!obs2)'}
sys_prog |= {'lot','X0reach'}

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
#if not ctrl.save('continuous_test1.png'):
print(ctrl)

# @synthesize_section_end@
cs = 'Sinit'
counter = 1
with open("out.txt","w") as f:
	while counter <= 20:
	        	nv,ns = random.choice(ctrl.adj[cs].items())
	        	pos = str(ns[0]['loc'])
	        	print(pos)
	        	print(disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(int(pos))])
	        	f.write(
	        		str(disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(int(pos))]))
	        	cs = nv

	        	counter += 1
# Simulation
#tomatlab.export('test1_2d_continuous.mat', ctrl, sys_dyn, disc_dynamics,
#disc_params)
