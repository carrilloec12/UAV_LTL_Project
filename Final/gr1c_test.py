"""
This example illustrates how to export a Mealy Machine to
Matlab/Simulink/Stateflow (see the very last line).

For detailed comments on the example itself,
please see examples/robot_planning/discrete.py
"""
from tulip import transys, spec, synth

# import file that contains to_stateflow
import sys
sys.path.append('../')
import tomatlab

#
# System dynamics
#
# The system is modeled as a discrete transition system in which the
# robot can be located anyplace on a 2x2 grid of cells.  Transitions
# between adjacent cells are allowed, which we model as a transition
# system in this example (it would also be possible to do this via a
# formula)
#
# We label the states using the following picture
#
#    
#     +----+----+
#     | X2 | X3 |
#     +----+----+
#     | X0 | X1 |
#     +----+----+

# Create a finite transition system
sys = transys.FTS()

# @system_dynamics_section@
# Create a finite transition system
sys = transys.FTS()

# Define the states of the system
sys.states.add_from(['X0','X1','X2','X3'])
sys.states.initial.add('X0')    # start in state X0

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys.transitions.add_comb({'X0'}, {'X1', 'X2'})
sys.transitions.add_comb({'X1'}, {'X0', 'X3'})
sys.transitions.add_comb({'X2'}, {'X0', 'X3'})
sys.transitions.add_comb({'X3'}, {'X1', 'X2'})
# @system_dynamics_section_end@

# @system_labels_section@
# Add atomic propositions to the states
sys.atomic_propositions.add_from({'goal','home'}) #, 'obsX1'}) 
sys.states.add('X0', ap={'home'})
sys.states.add('X3', ap={'goal'})
#sys.states.add('X1', ap={'obsX1'})

# Environment variables and specification
env_vars = {'park'}
env_init = set()
env_prog = '!park'
env_safe = set()

# System specification
sys_vars = {'X0reach'}
sys_init = {'X0reach'}
sys_prog = {'home'}
sys_safe = {'(X (X0reach) <-> goal) || (X0reach && !park)'}
sys_prog |= {'X0reach'}

# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

specs.moore = False
specs.plus_one = False
specs.qinit = '\A \E'

# Controller synthesis
ctrl = synth.synthesize('gr1c', specs, sys=sys)

print ctrl 
# Generate a MATLAB script that generates a Mealy Machine
tomatlab.export('robot_discrete.mat', ctrl)

# ts = {}
# ts['UAV'] = sys 
# #ts['UAV2'] = sys2
# ts['UAV'].owner = 'sys'
# #ts['UAV2'].owner = 'sys'

# bool_actions = {'UAV'} #,'UAV2'}
# ignore_init = {'UAV'} #,'UAV2'}

# assert isinstance(ts, dict)
# for name, t in ts.iteritems():
	
# 	assert isinstance(t, transys.FiniteTransitionSystem)
# 	ignore = name in ignore_init
# 	bool_act = name in bool_actions
# 	statevar = name
	
# 	print t
# 	if t.owner == 'sys':
# 		print "here"
# 		print "name", name
# 		print "ignore", ignore
# 		print "bool", bool_act
# 		print "statevar", statevar
        
# 		specs |= synth.sys_to_spec(t, ignore, statevar, bool_actions=bool_act)
# 		print specs
# 	