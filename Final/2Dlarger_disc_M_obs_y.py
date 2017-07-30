#!/usr/bin/env python
"""
Test Case #3:

UAV collision avoidance:

There are two obstacles. Initially, one obstacle is put in X1 and the other obstacle is put in X5. 
The two obstacles never occupy the same place. Obstacles can transition to X1, X5 or X6.
UAV is required to move from X0 to X8 and back from X8 to X0 infinitely often.

UAV goes to goal and comes back home.


"""
# RMM, 20 Jul 2013
#
# Note: This code is commented to allow components to be extracted into
# the tutorial that is part of the users manual.  Comments containing
# strings of the form @label@ are used for this purpose.

# @import_section@
# Import the packages that we need
import logging

from tulip import transys, spec, synth
# @import_section_end@


logging.basicConfig(level=logging.WARNING)
logging.getLogger('tulip.spec.lexyacc').setLevel(logging.WARNING)
logging.getLogger('tulip.synth').setLevel(logging.WARNING)
logging.getLogger('tulip.interfaces.omega').setLevel(logging.WARNING)


#
# System dynamics
#
# The system is modeled as a discrete transition system in which the
# robot can be located anyplace on a 3x3 grid of cells.  Transitions
# between adjacent cells are allowed, which we model as a transition
# system in this example (it would also be possible to do this via a
# formula)
#
# We label the states using the following picture
#
#     +----+----+----+
#     | X6 | X7 | X8 |
#     +----+----+----+
#     | X3 | X4 | X5 |
#     +----+----+----+
#     | X0 | X1 | X2 |
#     +----+----+----+

# @system_dynamics_section@
# Create a finite transition system
sys = transys.FTS()

# Define the states of the system
sys.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8'])
sys.states.initial.add('X0')    # start in state X0

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys.transitions.add_comb({'X0'}, {'X1', 'X3'})
sys.transitions.add_comb({'X1'}, {'X0', 'X4', 'X2'})
sys.transitions.add_comb({'X2'}, {'X1', 'X5'})
sys.transitions.add_comb({'X3'}, {'X0', 'X4', 'X6'})
sys.transitions.add_comb({'X4'}, {'X3', 'X1', 'X5', 'X7'})
sys.transitions.add_comb({'X5'}, {'X4', 'X2', 'X8'})
sys.transitions.add_comb({'X6'}, {'X7', 'X3'})
sys.transitions.add_comb({'X7'}, {'X4', 'X6', 'X8'})
sys.transitions.add_comb({'X8'}, {'X5', 'X7'})
# @system_dynamics_section_end@

# @system_labels_section@
# Add atomic propositions to the states
sys.atomic_propositions.add_from({'goal','obsX1','home','obsX5','obsX6'})
sys.states.add('X0', ap={'home'})
sys.states.add('X8', ap={'goal'})
sys.states.add('X1', ap={'obsX1'})
sys.states.add('X5', ap={'obsX5'})
sys.states.add('X6', ap={'obsX6'})
# @system_labels_section_end@

# if IPython and Matplotlib available
#sys.plot()


# Environment variables and specification1
# @environ_section@
env_vars = {'obs_a': range(3), 'obs_b': range(3)}
env_init = {'(obs_a = 0)', '(obs_b = 2)'}
env_prog = set() 
env_safe = {'((obs_a = 0) -> (X (obs_a = 1 || obs_a = 2)))', 
			'((obs_a = 1) -> (X (obs_a = 0 || obs_a = 2)))',
			'((obs_a = 2) -> (X (obs_a = 0 || obs_a = 1)))', 
			'((obs_b = 2) -> (X (obs_b = 1 || obs_b = 0)))', 
			'((obs_b = 1) -> (X (obs_b = 0 || obs_b = 2)))', 
			'((obs_b = 0) -> (X (obs_b = 1 || obs_b = 2)))', 
			'(obs_a != obs_b)'}


sys_vars = set()
sys_init = {'home'}
sys_prog = {'goal', 'home'}               # []<>home
sys_safe = {'((obs_a = 0) -> X (!obsX1))', '((obs_a = 1) -> X (!obsX6))', '((obs_a = 2) -> X (!obsX5))',
			'((obs_b = 0) -> X (!obsX1))', '((obs_b = 1) -> X (!obsX6))', '((obs_b = 2) -> X (!obsX5))'}
# @specs_setup_section_end@
#


# @specs_create_section@
# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
# @specs_create_section_end@

#
# Controller synthesis
#
# At this point we can synthesize the controller using one of the available
# methods.
#
# @synthesize@
# Moore machines
# controller reads `env_vars, sys_vars`, but not next `env_vars` values
specs.moore = True
# synthesizer should find initial system values that satisfy
# `env_init /\ sys_init` and work, for every environment variable
# initial values that satisfy `env_init`.
specs.qinit = '\E \A'
ctrl = synth.synthesize('omega', specs, sys=sys)
assert ctrl is not None, 'unrealizable'
# @synthesize_end@

#
# Generate a graphical representation of the controller for viewing,
# or a textual representation if pydot is missing.
#
# @plot_print@
#if not ctrl.save('discrete.png'):
#    print(ctrl)
# @plot_print_end@

print ctrl 
a = ctrl.transitions.find(from_states=[0], to_states=[1])
print a 