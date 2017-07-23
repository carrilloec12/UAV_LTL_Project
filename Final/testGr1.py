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

# UAV 1 
sys = transys.FTS()
sys.owner = 'sys'

# Define the states of the system
sys.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8'])
sys.states.initial.add('X0')    # start in state X0

# Define the allowable transitions
sys.transitions.add_comb({'X0'}, {'X1', 'X3'})
sys.transitions.add_comb({'X1'}, {'X0', 'X4', 'X2'})
sys.transitions.add_comb({'X2'}, {'X1', 'X5'})
sys.transitions.add_comb({'X3'}, {'X0', 'X4', 'X6'})
sys.transitions.add_comb({'X4'}, {'X3', 'X1', 'X5', 'X7'})
sys.transitions.add_comb({'X5'}, {'X4', 'X2', 'X8'})
sys.transitions.add_comb({'X6'}, {'X7', 'X3'})
sys.transitions.add_comb({'X7'}, {'X4', 'X6', 'X8'})
sys.transitions.add_comb({'X8'}, {'X5', 'X7'})

# Add atomic propositions to the states
sys.atomic_propositions.add_from({'home', 'goal'})
sys.states.add('X0', ap={'home'})
sys.states.add('X8', ap={'goal'})

# UAV 2
sys2 = transys.FTS()
sys2.owner = 'sys'

# Define the states of the system
sys2.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8'])
sys2.states.initial.add('X6')    # start in state X6

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys2.transitions.add_comb({'X0'}, {'X1', 'X3'})
sys2.transitions.add_comb({'X1'}, {'X0', 'X4', 'X2'})
sys2.transitions.add_comb({'X2'}, {'X1', 'X5'})
sys2.transitions.add_comb({'X3'}, {'X0', 'X4', 'X6'})
sys2.transitions.add_comb({'X4'}, {'X3', 'X1', 'X5', 'X7'})
sys2.transitions.add_comb({'X5'}, {'X4', 'X2', 'X8'})
sys2.transitions.add_comb({'X6'}, {'X7', 'X3'})
sys2.transitions.add_comb({'X7'}, {'X4', 'X6', 'X8'})
sys2.transitions.add_comb({'X8'}, {'X5', 'X7'})
# @system_dynamics_section_end@

# @system_labels_section@
# Add atomic propositions to the states
sys2.atomic_propositions.add_from({'goal', 'home'})
sys2.states.add('X0', ap={'home'})
sys2.states.add('X8', ap={'goal'})


# Environment variables and specification
env_vars = {'park'}
env_init = {'park'}
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

print specs 
specs.moore = False
specs.plus_one = False
specs.qinit = '\A \E'


# Controller synthesis for each individual UAV
# UAV 1
ctrl = synth.synthesize('gr1c', specs, sys=sys)
print ctrl 
# UAV 2
ctrl = synth.synthesize('gr1c', specs, sys=sys2)
print ctrl 

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
	print "iteration*******"
	print t 
	assert isinstance(t, transys.FiniteTransitionSystem)
	ignore = name in ignore_init
	bool_act = name in bool_actions
	statevar = name
	
	print t
	if t.owner == 'sys':
		# print "here"
		# print "name", name
		# print "ignore", ignore
		# print "bool", bool_act
		# print "statevar", statevar
        
		specs |= synth.sys_to_spec(t, ignore, statevar, bool_actions=bool_act)
		print specs
	elif t.owner == 'env':
		#print "here 2"
		specs |= synth.env_to_spec(t, ignore, statevar, bool_actions=bool_act)
		print specs

specs.moore = False
specs.plus_one = False
specs.qinit = '\A \E'


ctrl = gr1c.synthesize(specs)
print "ctrl", ctrl 
if not isinstance(ctrl, transys.MealyMachine):
	print None

#ctrl.remove_deadends()

# Generate a MATLAB script that generates a Mealy Machine
#tomatlab.export('robot_discrete.mat', ctrl)
