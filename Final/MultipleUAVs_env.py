"""
This example illustrates how to synthesize a controller for two UAV's, each defined as a FTS

"""
from tulip import transys, spec, synth
from tulip.interfaces import gr1c
import tomatlab
# import file that contains to_stateflow
import sys
sys.path.append('../')
#import tomatlab

# We label the states using the following picture
#

#     +----+----+
#     | X2 | X3 | 
#     +----+----+
#     | X0 | X1 | 
#     +----+----+

# UAV 1 
sys = transys.FTS()
sys.owner = 'sys'

# Define the states of the system
sys.states.add_from(['X0','X1','X2','X3'])
sys.states.initial.add('X0')    # start in state X0


# Define the allowable transitions
# sys.transitions.add_comb({'X0'}, {'X1', 'X2'})
# sys.transitions.add_comb({'X1'}, {'X0', 'X3'})
# sys.transitions.add_comb({'X2'}, {'X0', 'X3'})
# sys.transitions.add_comb({'X3'}, {'X1', 'X2'})

sys.transitions.add_comb({'X0'}, {'X3','X1','X2'})
sys.transitions.add_comb({'X1'}, {'X2','X3'})
sys.transitions.add_comb({'X2'}, {'X1','X0'})
sys.transitions.add_comb({'X3'}, {'X0','X2'})


# Add atomic propositions to the states
sys.atomic_propositions.add_from({'obsX0', 'obsX1', 'obsX2', 'obsX3'})
sys.states.add('X0', ap={'obsX0'})
sys.states.add('X1', ap={'obsX1'})
sys.states.add('X2', ap={'obsX2'})
sys.states.add('X3', ap={'obsX3'})

# UAV 2
sys2 = transys.FTS()
sys2.owner = 'sys'

# Define the states of the system
sys2.states.add_from(['X0','X1','X2','X3'])
sys2.states.initial.add('X2')    # start in state X6

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys2.transitions.add_comb({'X0'}, {'X1'})
sys2.transitions.add_comb({'X1'}, {'X0', 'X3'})
sys2.transitions.add_comb({'X2'}, {'X3'})
sys2.transitions.add_comb({'X3'}, {'X1', 'X2'})
# @system_dynamics_section_end@

# @system_labels_section@
# Add atomic propositions to the states
sys2.atomic_propositions.add_from({'obsX0', 'obsX1', 'obsX2', 'obsX3'})
sys2.states.add('X0', ap={'obsX0'})
sys2.states.add('X1', ap={'obsX1'})
sys2.states.add('X2', ap={'obsX2'})
sys2.states.add('X3', ap={'obsX3'})

env0 = transys.FTS()
env0.owner = 'env'
env0.states.add_from({'X0', 'X1', 'X2', 'X3'})
env0.states.initial.add('X0')

# env0.atomic_propositions.add_from({'obsX0', 'obsX1', 'obsX2', 'obsX3'}) #, 'obsX1', 'obsX2', 'obsX3'
# env0.states.add('X0', ap={'obsX0'})
# env0.states.add('X1', ap={'obsX1'})
# env0.states.add('X2', ap={'obsX2'})
# env0.states.add('X3', ap={'obsX3'})

env0.transitions.add_from([
    ('X0', 'X1'), ('X1', 'X2'), ('X2', 'X3'), ('X3', 'X0')
])


# Environment variables and specification
env_vars = set()
env_init = set()
env_prog = ''
env_safe = set()

# System specification
sys_vars = set()
sys_init = set()
sys_prog = set()
sys_safe = set()
sys_prog |= {'obsX3'}

# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

print "Specs 1"
print specs 
specs.moore = False
specs.plus_one = False
specs.qinit = '\A \E'


# # Controller synthesis for each individual UAV
# # UAV 1
# ctrl = synth.synthesize('gr1c', specs, sys=sys, env=env0)
# #print ctrl 
# # UAV 2
# ctrl = synth.synthesize('gr1c', specs, sys=sys2, env=env0)
# #print ctrl 
# #print "Gr1c", ctrl 
# ctrl = synth.synthesize('omega', specs, sys=sys, env=env0)
# #print "Omega", ctrl 


# Controller synthesis for both UAVs
ts = {}
ts['UAV'] = sys 
ts['UAV2'] = sys2
ts['UAV'].owner = 'sys'
ts['UAV2'].owner = 'sys'


bool_actions = {'UAV','UAV2'}
ignore_init = {'UAV','UAV2'}


#ctrl = synth.synthesize_many(specs, ts, ignore_init, bool_actions, solver='gr1c')
specs.moore = True
specs.plus_one = True
specs.qinit = '\A \A'

assert isinstance(ts, dict)
for name, t in ts.iteritems():
	#print "iteration*******"
	#print t 
	assert isinstance(t, transys.FiniteTransitionSystem)
	ignore = name in ignore_init
	bool_act = name in bool_actions
	statevar = name
	
	#print t
	if t.owner == 'sys':
		# print "here"
		# print "name", name
		# print "ignore", ignore
		# print "bool", bool_act
		# print "statevar", statevar
		# print "Iteration Begin"
		# a =  synth.sys_to_spec(t, ignore, statevar, bool_actions=bool_act)
		# print a
		# a |= '&& [](X(!(UAV = UAV2)))'
		# print a
		# print "Iteration End"
		specs |= synth.sys_to_spec(t, ignore, statevar, bool_actions=bool_act)
		#print specs
	elif t.owner == 'env':
		#print "here 2"
		specs |= synth.env_to_spec(t, ignore, statevar, bool_actions=bool_act)
		#print specs

print "Results"
print specs


#specs |= '&& [](X(!(UAV = UAV2)))'
# print specs
specs.moore = True
# synthesizer should find initial system values that satisfy
# `env_init /\ sys_init` and work, for every environment variable
# initial values that satisfy `env_init`.
specs.qinit = '\E \A'
ctrl = synth.synthesize('omega', specs, env=env0)
assert ctrl is not None, 'unrealizable'

print ctrl
# specs.moore = False
# specs.plus_one = False
# specs.qinit = '\A \E'


# ctrl = gr1c.synthesize(specs)
# print "ctrl", ctrl 
# if not isinstance(ctrl, transys.MealyMachine):
# 	print None

#ctrl.remove_deadends()

# Generate a MATLAB script that generates a Mealy Machine
#tomatlab.export('robot_discrete.mat', ctrl)
