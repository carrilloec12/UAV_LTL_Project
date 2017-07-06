#!/usr/bin/env python
"""
UAV collision avoidance:

Initially, obstacle is put in X1. Then, obstacle is removed from X1 and changed to X4.
UAV is required to move from X0 to X4 and back from X4 to X0 infinitely often.

UAV goes to goal and comes back home.

count = number of steps that obs has been true.

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
from tulip.interfaces import gr1c
from tulip.interfaces import gr1py
from tulip.interfaces import jtlv
from tulip.interfaces import omega as omega_int
try:
    from tulip.interfaces import slugs
except ImportError:
    slugs = None
from tulip.spec import GRSpec
from tulip import transys
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
sys.owner = 'sys'
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
sys.atomic_propositions.add_from({'goal'})
# sys.states.add('X0', ap={'home'})
sys.states.add('X8', ap={'goal'})
# sys.states.add('X1', ap={'obsX1'})
# sys.states.add('X3', ap={'obsX3'})
# @system_labels_section_end@

# if IPython and Matplotlib available
#sys.plot()

sys2 = transys.FTS()
sys2.owner = 'sys'

# Define the states of the system
sys2.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8'])
sys2.states.initial.add('X6')    # start in state X0

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
sys2.atomic_propositions.add_from({'goal'})
# sys.states.add('X0', ap={'home'})
sys2.states.add('X8', ap={'goal'})


# sys.states.add('X1', ap={'obsX1'})
# sys.states.add('X3', ap={'obsX3'})
# @system_labels_section_end@

# if IPython and Matplotlib available
#sys.plot()

#
ts = {}
ts['UAV'] = sys 
ts['UAV2'] = sys2
ts['UAV'].owner = 'sys'
ts['UAV2'].owner = 'sys'



#
# Environment variables and specification
# @environ_section@
env_vars = {}  
env_init = {}    
env_prog = {} 
env_safe = set() 

# {'(!obs3 ->  (X count3 = count3))', '(obs3 ->  (X count3 = count3 + 1))', 'count3<2',
# 			'(!obs1 ->  (X count1 = count1))', '(obs1 ->  (X count1 = count1 + 1))', 'count1<2'} 
# @environ_section_end@

# @specs_setup_section@
# Augment the system description to make it GR(1)
#! TODO: create a function to convert this type of spec automatically
sys_vars = {'X0reach'}          # infer the rest from TS
sys_init = {'X0reach'}
sys_prog = {'goal'}             # []<>goal
sys_safe = set() #{'((obs3 -> X !obsX3)||(obs1 -> X !obsX1))'} 
sys_prog |= set() #{'UAV'}
# @specs_setup_section_end@

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

# # synthesizer should find initial system values that satisfy
# # `env_init /\ sys_init` and work, for every environment variable
# # initial values that satisfy `env_init`.
# # specs, ts=None, ignore_init=None,
# #                     bool_actions=None, solver='gr1c'
# #specs.qinit = '\E \A'
# # 

bool_actions = {'UAV','UAV2'}
ignore_init = {'UAV','UAV2'}

assert isinstance(ts, dict)
print "Beginning process"
print isinstance(ts, dict)

for name, t in ts.iteritems():
	print "name", name
	assert isinstance(t, transys.FiniteTransitionSystem)
	ignore = name in ignore_init
	print "ignore", ignore
	bool_act = name in bool_actions
	print "bool", bool_act
	statevar = name
	print "statevar", statevar
	if t.owner == 'sys':
		print "here"
		specs |= synth.sys_to_spec(t, ignore, statevar, bool_actions=bool_act)
		print specs
	elif t.owner == 'env':
		print "here 2"
		specs |= synth.env_to_spec(t, ignore, statevar, bool_actions=bool_act)

specs.moore = False
specs.plus_one = False
specs.sys_init = False

solver = 'gr1c'
if solver == 'gr1c':
	print "gr1c"
	ctrl = gr1c.synthesize(specs)
elif solver == 'slugs':
    if slugs is None:
        raise ValueError('Import of slugs interface failed. ' +
                         'Please verify installation of "slugs".')
    ctrl = slugs.synthesize(specs)
elif solver == 'jtlv':
    ctrl = jtlv.synthesize(specs)
else:
    raise Exception('Unknown solver: ' + str(solver) + '. '
                    'Available solvers: "jtlv", "gr1c", and "slugs"')
try:
    logger.debug('Mealy machine has: n = ' +
                 str(len(ctrl.states)) + ' states.')
except:
    logger.debug('No Mealy machine returned.')
# no controller found ?
# counterstrategy not constructed by synthesize
# if not isinstance(ctrl, transys.MealyMachine):
#     return None
# ctrl.remove_deadends()

# for name, t in ts.iteritems():
#     #assert isinstance(t, transys.FiniteTransitionSystem)
#     print name
#     print t

#     print name in ignoreInit

#     # print ignore 

#     print name in boolActions
    
#     # print ignore 

# ctrl = synth.synthesize_many(specs, ts, ignoreInit, boolActions, solver='gr1c')


# #ctrl = synth.synthesize('omega', specs, sys=sys)
# #assert ctrl is not None, 'unrealizable'
# # @synthesize_end@

# #
# # Generate a graphical representation of the controller for viewing,
# # or a textual representation if pydot is missing.
# #
# # @plot_print@
# #if not ctrl.save('discrete.png'):
# #    print(ctrl)
# # @plot_print_end@

# # print ctrl 
# # a = ctrl.transitions.find(from_states=[0], to_states=[1])
# # print a 