"""
This example illustrates how to synthesize a controller for two UAV's, each defined as a FTS

"""
from tulip import transys, spec, synth
from tulip.interfaces import gr1c
import tomatlab
# import file that contains to_stateflow
import sys
import time
sys.path.append('../')
#import tomatlab

#     3D Model:

#     1st level(lower):
#     y axis
#     +---+---+---+
#     | 2 | 5 | 8 |
#     +---+---+---+
#     | 1 | 4 | 7 |
#     +---+---+---+
#     | 0 | 3 | 6 |
#     +---+---+---+ x axis

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
#     +----+----+----+
#     | 20 | 23 | 26 |
#     +----+----+----+
#     | 19 | 22 | 25 |
#     +----+----+----+
#     | 18 | 21 | 24 |
#     +----+----+----+ x axis

# UAV 1 
sys = transys.FTS()
sys.owner = 'sys'

# Define the states of the system
sys.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8',
					 'X9','X10','X11','X12','X13','X14','X15','X16','X17',
					 'X18','X19','X20','X21','X22','X23','X24','X25','X26'])
sys.states.initial.add('X0')    # start in state X0

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys.transitions.add_comb({'X0'}, {'X1','X3','X9','X4'})
sys.transitions.add_comb({'X1'}, {'X0','X2','X4','X10','X3','X5'})
sys.transitions.add_comb({'X2'}, {'X1','X5','X11','X4'})
sys.transitions.add_comb({'X3'}, {'X0','X6','X4','X12','X7','X1'})
sys.transitions.add_comb({'X4'}, {'X1','X3','X5','X7','X13','X8','X2','X0','X6'})
sys.transitions.add_comb({'X5'}, {'X2','X4','X8','X14','X1','X7'})
sys.transitions.add_comb({'X6'}, {'X3','X7','X15','X4'})
sys.transitions.add_comb({'X7'}, {'X4','X6','X8','X16','X3','X5'})
sys.transitions.add_comb({'X8'}, {'X5','X7','X17','X4'})
sys.transitions.add_comb({'X9'}, {'X0','X10','X12','X18','X13'})
sys.transitions.add_comb({'X10'}, {'X1','X9','X11','X13','X19','X14','X12'})
sys.transitions.add_comb({'X11'}, {'X2','X10','X14','X20','X13'})
sys.transitions.add_comb({'X12'}, {'X3','X9','X13','X15','X21','X10','X16'})
sys.transitions.add_comb({'X13'}, {'X4','X10','X12','X14','X16','X22','X11','X17','X9','X15'})
sys.transitions.add_comb({'X14'}, {'X5','X11','X13','X17','X23','X10','X16'})
sys.transitions.add_comb({'X15'}, {'X6','X12','X16','X24','X13'})
sys.transitions.add_comb({'X16'}, {'X7','X13','X15','X17','X25','X12','X14'})
sys.transitions.add_comb({'X17'}, {'X8','X14','X16','X26','X13'})
sys.transitions.add_comb({'X18'}, {'X9','X19','X21','X22'})
sys.transitions.add_comb({'X19'}, {'X10','X18','X20','X22','X21','X23'})
sys.transitions.add_comb({'X20'}, {'X11','X19','X23','X22'})
sys.transitions.add_comb({'X21'}, {'X12','X18','X22','X24','X19','X25'})
sys.transitions.add_comb({'X22'}, {'X13','X19','X21','X23','X25','X20','X26','X18','X24'})
sys.transitions.add_comb({'X23'}, {'X14','X20','X22','X26','X19','X25'})
sys.transitions.add_comb({'X24'}, {'X15','X21','X25','X22'})
sys.transitions.add_comb({'X25'}, {'X16','X22','X24','X26','X21','X23'})
sys.transitions.add_comb({'X26'}, {'X17','X23','X25','X22'})
# @system_dynamics_section_end@

# Add atomic propositions to the states
sys.atomic_propositions.add_from({'home', 'goal', 'sX6', 'sX13', 'sX20', 'sX22', 'sX25','sX17'})
sys.states.add('X0', ap={'home'})
sys.states.add('X26', ap={'goal'})
sys.states.add('X6', ap={'sX6'})
sys.states.add('X13', ap={'sX13'})
sys.states.add('X20', ap={'sX20'})
sys.states.add('X25', ap={'sX25'})
sys.states.add('X22', ap={'sX22'})
sys.states.add('X17', ap={'sX17'})

# UAV 2
sys2 = transys.FTS()
sys2.owner = 'sys'

# Define the states of the system
sys2.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8',
					 'X9','X10','X11','X12','X13','X14','X15','X16','X17',
					 'X18','X19','X20','X21','X22','X23','X24','X25','X26'])
sys2.states.initial.add('X1')    # start in state X0

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys2.transitions.add_comb({'X0'}, {'X1','X3','X9','X4'})
sys2.transitions.add_comb({'X1'}, {'X0','X2','X4','X10','X3','X5'})
sys2.transitions.add_comb({'X2'}, {'X1','X5','X11','X4'})
sys2.transitions.add_comb({'X3'}, {'X0','X6','X4','X12','X7','X1'})
sys2.transitions.add_comb({'X4'}, {'X1','X3','X5','X7','X13','X8','X2','X0','X6'})
sys2.transitions.add_comb({'X5'}, {'X2','X4','X8','X14','X1','X7'})
sys2.transitions.add_comb({'X6'}, {'X3','X7','X15','X4'})
sys2.transitions.add_comb({'X7'}, {'X4','X6','X8','X16','X3','X5'})
sys2.transitions.add_comb({'X8'}, {'X5','X7','X17','X4'})
sys2.transitions.add_comb({'X9'}, {'X0','X10','X12','X18','X13'})
sys2.transitions.add_comb({'X10'}, {'X1','X9','X11','X13','X19','X14','X12'})
sys2.transitions.add_comb({'X11'}, {'X2','X10','X14','X20','X13'})
sys2.transitions.add_comb({'X12'}, {'X3','X9','X13','X15','X21','X10','X16'})
sys2.transitions.add_comb({'X13'}, {'X4','X10','X12','X14','X16','X22','X11','X17','X9','X15'})
sys2.transitions.add_comb({'X14'}, {'X5','X11','X13','X17','X23','X10','X16'})
sys2.transitions.add_comb({'X15'}, {'X6','X12','X16','X24','X13'})
sys2.transitions.add_comb({'X16'}, {'X7','X13','X15','X17','X25','X12','X14'})
sys2.transitions.add_comb({'X17'}, {'X8','X14','X16','X26','X13'})
sys2.transitions.add_comb({'X18'}, {'X9','X19','X21','X22'})
sys2.transitions.add_comb({'X19'}, {'X10','X18','X20','X22','X21','X23'})
sys2.transitions.add_comb({'X20'}, {'X11','X19','X23','X22'})
sys2.transitions.add_comb({'X21'}, {'X12','X18','X22','X24','X19','X25'})
sys2.transitions.add_comb({'X22'}, {'X13','X19','X21','X23','X25','X20','X26','X18','X24'})
sys2.transitions.add_comb({'X23'}, {'X14','X20','X22','X26','X19','X25'})
sys2.transitions.add_comb({'X24'}, {'X15','X21','X25','X22'})
sys2.transitions.add_comb({'X25'}, {'X16','X22','X24','X26','X21','X23'})
sys2.transitions.add_comb({'X26'}, {'X17','X23','X25','X22'})
# @system_dynamics_section_end@

# @system_labels_section@
# Add atomic propositions to the states
sys2.atomic_propositions.add_from({'goal', 'home', 's2X6', 's2X13', 's2X20', 's2X22', 's2X25','s2X17'})
sys2.states.add('X1', ap={'home'})
sys2.states.add('X26', ap={'goal'})
sys2.states.add('X6', ap={'s2X6'})
sys2.states.add('X13', ap={'s2X13'})
sys2.states.add('X20', ap={'s2X20'})
sys2.states.add('X22', ap={'s2X22'})
sys2.states.add('X25', ap={'s2X25'})
sys2.states.add('X17', ap={'s2X17'})

env0 = transys.FTS()
env0.owner = 'env'
env0.states.add_from({'X6', 'X13', 'X20'})
env0.states.initial.add('X6')

env0.atomic_propositions.add_from({'obsX6', 'obsX13', 'obsX20'})
env0.states.add('X6', ap={'obsX6'})
env0.states.add('X13', ap={'obsX13'})
env0.states.add('X20', ap={'obsX20'})

env0.transitions.add_from([
    ('X6', 'X13'), ('X13', 'X20'), ('X20', 'X13'), ('X13', 'X6')
])
# Environment variables and specification
env_vars = set()
env_init = set()
env_prog = ''
env_safe = set()

# System specification
sys_vars = set()
sys_init = set()
sys_prog = {'home','goal'}
#sys_safe = set() 
sys_safe =  {'((obsX6) -> X (!sX6))', '((obsX6) -> X (!s2X6))',
			'((obsX13) -> X (!sX13))', '((obsX13) -> X (!s2X13))',
 			'((obsX20) -> X (!sX20))', '((obsX20) -> X (!s2X20))',
 			'!(s2X25 && sX25)', '!(s2X22 && sX22)', '!(s2X17 && sX17)'}
# sys_safe = {'((obsX6) -> X (!sX6))',
# 			'((obsX13) -> X (!sX13))',
# 			'((obsX20) -> X (!sX20))'}
sys_prog |= {'goal'}

# Create the specification
specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

# print "Specs 1"
# print specs 
specs.moore = False
specs.plus_one = False
specs.qinit = '\A \E'


# Controller synthesis for each individual UAV
# UAV 1
# ctrl = synth.synthesize('gr1c', specs, sys=sys, env=env0)
# print ctrl 
# # UAV 2
# ctrl = synth.synthesize('gr1c', specs, sys=sys2, env=env0)
# print ctrl 
# # #print "Gr1c", ctrl 

# ctrl = synth.synthesize('omega', specs, sys=sys, env=env0)
# print "Omega", ctrl 

# ctrl = synth.synthesize('omega', specs, sys=sys2, env=env0)
# print "Omega", ctrl 

# Controller synthesis for both UAVs
ts = {}
ts['UAV'] = sys 
ts['UAV2'] = sys2
ts['ENV0'] = env0 
ts['UAV'].owner = 'sys'
ts['UAV2'].owner = 'sys'
ts['ENV0'].owner = 'env'


bool_actions = {'UAV','UAV2'}
ignore_init = {'UAV','UAV2'}


# ctrl = synth.synthesize_many(specs, ts, ignore_init, bool_actions, solver='gr1c')
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

# print "Results"
# print specs


# #specs |= '&& [](X(!(UAV = UAV2)))'
# # print specs
specs.moore = True
# # synthesizer should find initial system values that satisfy
# # `env_init /\ sys_init` and work, for every environment variable
# # initial values that satisfy `env_init`.
specs.qinit = '\E \A'
start = time.time()
ctrl = synth.synthesize('omega', specs)
print 'It took', time.time()-start, 'seconds.'
assert ctrl is not None, 'unrealizable'
print(ctrl)

# if not ctrl.save('MultipleUAVs.png'):
#     print(ctrl)

#print ctrl
# # specs.moore = False
# # specs.plus_one = False
# # specs.qinit = '\A \E'


# # ctrl = gr1c.synthesize(specs)
# # print "ctrl", ctrl 
# # if not isinstance(ctrl, transys.MealyMachine):
# # 	print None

# #ctrl.remove_deadends()

# # Generate a MATLAB script that generates a Mealy Machine
# #tomatlab.export('robot_discrete.mat', ctrl)
