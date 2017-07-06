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
uncertainty = 0.1

# Continuous state space
cont_state_space = box2poly([[0., 3.], [0., 3.], [0., 3.]])

# Continuous dynamics
A = np.array([[1.0, 0., 0.], [ 0., 1.0, 0.], [0., 0., 1.0]])
B = np.array([[0.5, 0., 0.], [ 0., 0.5, 0.], [0., 0., 0.5]])
E = np.array([[1,0,0], [0,1,0], [0,0,1]])

# Available control, possible disturbances
U = input_bound *np.array([[-1., 1.], [-1., 1.], [-1., 1.]])
W = uncertainty *np.array([[-1., 1.], [-1., 1.], [-1., 1.]])


# Convert to polyhedral representation
U = box2poly(U)
W = box2poly(W)

# Construct the LTI system describing the dynamics
sys_dyn = hybrid.LtiSysDyn(A, B, E, None, U, W, cont_state_space)
# @dynamics_section_end@
#     3D Model:

#     1st level(lower):
#     y axis
#     +------+---+---+
#     |  2   | 5 | 8 |
#     +------+---+---+
#     |  1   | 4 | 7 |
#     +------+---+---+
#     | home | 3 | 6 |
#     +------+---+---+ x axis

#     2nd level(middle):
#     y axis
#     +----+----+----+
#     | 11 | 14 | 17 |
#     +----+----+----+
#     | 10 | 13 | 16 |
#     +----+----+----+
#     | 9  | 12 | 15 |
#     +----+----+----+ x axis

#     3rd level(upper):
#     y axis
#     +----+----+-------------+
#     | 20 | 23 | destination |
#     +----+----+-------------+
#     | 19 | 22 |     25      |
#     +----+----+-------------+
#     | 18 | 21 |     24      |
#     +----+----+-------------+ x axis
# @partition_section@
# Define atomic propositions for relevant regions of state space
cont_props = {}
cont_props['home'] = box2poly([[0., 1.], [0., 1.], [0., 1.]])
cont_props['destination'] = box2poly([[2., 3.], [2., 3.], [2., 3.]])
cont_props['obs1'] = box2poly([[0., 1.], [1., 2.], [0., 1.]])
cont_props['obs2'] = box2poly([[0., 1.], [2., 3.], [0., 1.]])
cont_props['obs3'] = box2poly([[1., 2.], [0., 1.], [0., 1.]])
cont_props['obs4'] = box2poly([[1., 2.], [1., 2.], [0., 1.]])
cont_props['obs5'] = box2poly([[1., 2.], [2., 3.], [0., 1.]])
cont_props['obs6'] = box2poly([[2., 3.], [0., 1.], [0., 1.]])
cont_props['obs7'] = box2poly([[2., 3.], [1., 2.], [0., 1.]])
cont_props['obs8'] = box2poly([[2., 3.], [2., 3.], [0., 1.]])
cont_props['obs9'] = box2poly([[0., 1.], [0., 1.], [1., 2.]])
cont_props['obs10'] = box2poly([[0., 1.], [1., 2.], [1., 2.]])
cont_props['obs11'] = box2poly([[0., 1.], [2., 3.], [1., 2.]])
cont_props['obs12'] = box2poly([[1., 2.], [0., 1.], [1., 2.]])
cont_props['obs13'] = box2poly([[1., 2.], [1., 2.], [1., 2.]])
cont_props['obs14'] = box2poly([[1., 2.], [2., 3.], [1., 2.]])
cont_props['obs15'] = box2poly([[2., 2.], [0., 1.], [1., 2.]])
cont_props['obs16'] = box2poly([[2., 3.], [1., 2.], [1., 2.]])
cont_props['obs17'] = box2poly([[2., 3.], [2., 3.], [1., 2.]])
cont_props['obs18'] = box2poly([[0., 1.], [0., 1.], [2., 3.]])
cont_props['obs19'] = box2poly([[0., 1.], [1., 2.], [2., 3.]])
cont_props['obs20'] = box2poly([[0., 1.], [2., 3.], [2., 3.]])
cont_props['obs21'] = box2poly([[1., 2.], [0., 1.], [2., 3.]])
cont_props['obs22'] = box2poly([[1., 2.], [1., 2.], [2., 3.]])
cont_props['obs23'] = box2poly([[1., 2.], [2., 3.], [2., 3.]])
cont_props['obs24'] = box2poly([[2., 3.], [0., 1.], [2., 3.]])
cont_props['obs25'] = box2poly([[2., 3.], [1., 2.], [2., 3.]])

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

env_vars = {'obs_a': range(1,26), 'obs_b': range(1,26),}
env_init = {'(obs_a = 6)', '(obs_b = 10)'}
env_prog = {'(obs_a = 20)', '(obs_b = 16)'}
env_safe = {'((obs_a = 1) -> (X obs_a = 2) || (X obs_a = 4) || (X obs_a = 10))',
            '((obs_a = 2) -> (X obs_a = 1) || (X obs_a = 5) || (X obs_a = 11))',
            '((obs_a = 3) -> (X obs_a = 4) || (X obs_a = 6) || (X obs_a = 12))',
            '((obs_a = 4) -> (X obs_a = 1) || (X obs_a = 3) || (X obs_a = 5) || (X obs_a = 7) || (X obs_a = 13))',
            '((obs_a = 5) -> (X obs_a = 2) || (X obs_a = 4) || (X obs_a = 8) || (X obs_a = 14))',
            '((obs_a = 6) -> (X obs_a = 3) || (X obs_a = 7) || (X obs_a = 15))',
            '((obs_a = 7) -> (X obs_a = 4) || (X obs_a = 6) || (X obs_a = 8) || (X obs_a = 16))',
            '((obs_a = 8) -> (X obs_a = 5) || (X obs_a = 7) || (X obs_a = 17))',
            '((obs_a = 9) -> (X obs_a = 10) || (X obs_a = 12) || (X obs_a = 18))',
            '((obs_a = 10) -> (X obs_a = 1) || (X obs_a = 9) || (X obs_a = 11) || (X obs_a = 13) || (X obs_a = 19))',
            '((obs_a = 11) -> (X obs_a = 2) || (X obs_a = 10) || (X obs_a = 14) || (X obs_a = 20))',
            '((obs_a = 12) -> (X obs_a = 3) || (X obs_a = 9) || (X obs_a = 13) || (X obs_a = 15) || (X obs_a = 21))',
            '((obs_a = 13) -> (X obs_a = 4) || (X obs_a = 10) || (X obs_a = 12) || (X obs_a = 14) || (X obs_a = 16) || (X obs_a = 22))',
            '((obs_a = 14) -> (X obs_a = 5) || (X obs_a = 11) || (X obs_a = 13) || (X obs_a = 17) || (X obs_a = 23))',
            '((obs_a = 15) -> (X obs_a = 6) || (X obs_a = 12) || (X obs_a = 16) || (X obs_a = 24))',
            '((obs_a = 16) -> (X obs_a = 7) || (X obs_a = 13) || (X obs_a = 15) || (X obs_a = 17) || (X obs_a = 25))',
            '((obs_a = 17) -> (X obs_a = 8) || (X obs_a = 14) || (X obs_a = 16))',
            '((obs_a = 18) -> (X obs_a = 9) || (X obs_a = 19) || (X obs_a = 21))',
            '((obs_a = 19) -> (X obs_a = 10) || (X obs_a = 18) || (X obs_a = 20) || (X obs_a = 22))',
            '((obs_a = 20) -> (X obs_a = 11) || (X obs_a = 19) || (X obs_a = 23))',
            '((obs_a = 21) -> (X obs_a = 12) || (X obs_a = 18) || (X obs_a = 22) || (X obs_a = 24))',
            '((obs_a = 22) -> (X obs_a = 13) || (X obs_a = 19) || (X obs_a = 21) || (X obs_a = 23) || (X obs_a = 25))',
            '((obs_a = 23) -> (X obs_a = 14) || (X obs_a = 20) || (X obs_a = 22))',
            '((obs_a = 24) -> (X obs_a = 15) || (X obs_a = 21) || (X obs_a = 25))',
            '((obs_a = 25) -> (X obs_a = 16) || (X obs_a = 22) || (X obs_a = 24))',

            '((obs_b = 1) -> (X obs_b = 2) || (X obs_b = 4) || (X obs_b = 10))',
            '((obs_b = 2) -> (X obs_b = 1) || (X obs_b = 5) || (X obs_b = 11))',
            '((obs_b = 3) -> (X obs_b = 4) || (X obs_b = 6) || (X obs_b = 12))',
            '((obs_b = 4) -> (X obs_b = 1) || (X obs_b = 3) || (X obs_b = 5) || (X obs_b = 7) || (X obs_b = 13))',
            '((obs_b = 5) -> (X obs_b = 2) || (X obs_b = 4) || (X obs_b = 8) || (X obs_b = 14))',
            '((obs_b = 6) -> (X obs_b = 3) || (X obs_b = 7) || (X obs_b = 15))',
            '((obs_b = 7) -> (X obs_b = 4) || (X obs_b = 6) || (X obs_b = 8) || (X obs_b = 16))',
            '((obs_b = 8) -> (X obs_b = 5) || (X obs_b = 7) || (X obs_b = 17))',
            '((obs_b = 9) -> (X obs_b = 10) || (X obs_b = 12) || (X obs_b = 18))',
            '((obs_b = 10) -> (X obs_b = 1) || (X obs_b = 9) || (X obs_b = 11) || (X obs_b = 13) || (X obs_b = 19))',
            '((obs_b = 11) -> (X obs_b = 2) || (X obs_b = 10) || (X obs_b = 14) || (X obs_b = 20))',
            '((obs_b = 12) -> (X obs_b = 3) || (X obs_b = 9) || (X obs_b = 13) || (X obs_b = 15) || (X obs_b = 21))',
            '((obs_b = 13) -> (X obs_b = 4) || (X obs_b = 10) || (X obs_b = 12) || (X obs_b = 14) || (X obs_b = 16) || (X obs_b = 22))',
            '((obs_b = 14) -> (X obs_b = 5) || (X obs_b = 11) || (X obs_b = 13) || (X obs_b = 17) || (X obs_b = 23))',
            '((obs_b = 15) -> (X obs_b = 6) || (X obs_b = 12) || (X obs_b = 16) || (X obs_b = 24))',
            '((obs_b = 16) -> (X obs_b = 7) || (X obs_b = 13) || (X obs_b = 15) || (X obs_b = 17) || (X obs_b = 25))',
            '((obs_b = 17) -> (X obs_b = 8) || (X obs_b = 14) || (X obs_b = 16))',
            '((obs_b = 18) -> (X obs_b = 9) || (X obs_b = 19) || (X obs_b = 21))',
            '((obs_b = 19) -> (X obs_b = 10) || (X obs_b = 18) || (X obs_b = 20) || (X obs_b = 22))',
            '((obs_b = 20) -> (X obs_b = 11) || (X obs_b = 19) || (X obs_b = 23))',
            '((obs_b = 21) -> (X obs_b = 12) || (X obs_b = 18) || (X obs_b = 22) || (X obs_b = 24))',
            '((obs_b = 22) -> (X obs_b = 13) || (X obs_b = 19) || (X obs_b = 21) || (X obs_b = 23) || (X obs_b = 25))',
            '((obs_b = 23) -> (X obs_b = 14) || (X obs_b = 20) || (X obs_b = 22))',
            '((obs_b = 24) -> (X obs_b = 15) || (X obs_b = 21) || (X obs_b = 25))',
            '((obs_b = 25) -> (X obs_b = 16) || (X obs_b = 22) || (X obs_b = 24))',

            '(obs_a != obs_b)'
            }

# System variables and requirements
sys_vars = set()
sys_init = {'home'}
sys_prog = {'destination'}               # []<>home
sys_safe = {'((obs_a = 1) -> X (!obs1))',
            '((obs_a = 2) -> X (!obs2))',
            '((obs_a = 3) -> X (!obs3))',
            '((obs_a = 4) -> X (!obs4))',
            '((obs_a = 5) -> X (!obs5))',
            '((obs_a = 6) -> X (!obs6))',
            '((obs_a = 7) -> X (!obs7))',
            '((obs_a = 8) -> X (!obs8))',
            '((obs_a = 9) -> X (!obs9))',
            '((obs_a = 10) -> X (!obs10))',
            '((obs_a = 11) -> X (!obs11))',
            '((obs_a = 12) -> X (!obs12))',
            '((obs_a = 13) -> X (!obs13))',
            '((obs_a = 14) -> X (!obs14))',
            '((obs_a = 15) -> X (!obs15))',
            '((obs_a = 16) -> X (!obs16))',
            '((obs_a = 17) -> X (!obs17))',
            '((obs_a = 18) -> X (!obs18))',
            '((obs_a = 19) -> X (!obs19))',
            '((obs_a = 20) -> X (!obs20))',
            '((obs_a = 21) -> X (!obs21))',
            '((obs_a = 22) -> X (!obs22))',
            '((obs_a = 23) -> X (!obs23))',
            '((obs_a = 24) -> X (!obs24))',
            '((obs_a = 25) -> X (!obs25))',

            '((obs_b = 1) -> X (!obs1))',
            '((obs_b = 2) -> X (!obs2))',
            '((obs_b = 3) -> X (!obs3))',
            '((obs_b = 4) -> X (!obs4))',
            '((obs_b = 5) -> X (!obs5))',
            '((obs_b = 6) -> X (!obs6))',
            '((obs_b = 7) -> X (!obs7))',
            '((obs_b = 8) -> X (!obs8))',
            '((obs_b = 9) -> X (!obs9))',
            '((obs_b = 10) -> X (!obs10))',
            '((obs_b = 11) -> X (!obs11))',
            '((obs_b = 12) -> X (!obs12))',
            '((obs_b = 13) -> X (!obs13))',
            '((obs_b = 14) -> X (!obs14))',
            '((obs_b = 15) -> X (!obs15))',
            '((obs_b = 16) -> X (!obs16))',
            '((obs_b = 17) -> X (!obs17))',
            '((obs_b = 18) -> X (!obs18))',
            '((obs_b = 19) -> X (!obs19))',
            '((obs_b = 20) -> X (!obs20))',
            '((obs_b = 21) -> X (!obs21))',
            '((obs_b = 22) -> X (!obs22))',
            '((obs_b = 23) -> X (!obs23))',
            '((obs_b = 24) -> X (!obs24))',
            '((obs_b = 25) -> X (!obs25))'
            }
sys_prog |= {'destination'}

# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
specs.qinit = r'\A \E'

# @synthesize_section@
# Synthesize
ctrl = synth.synthesize('omega', specs,
                        sys=disc_dynamics.ts, ignore_sys_init=True)
assert ctrl is not None, 'unrealizable'


# Generate a graphical representation of the controller for viewing
# if not ctrl.save('uav_continuous.png'):
#     print(ctrl)
# @synthesize_section_end@

#print(disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(2)])
#print(disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(10)])
#print(disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(4)])
#print(disc_dynamics.ppp.regions[disc_dynamics.ppp2ts.index(1)])

