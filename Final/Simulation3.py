"""
example to illustrate the combined use of
an environment and a system transition system.
"""
import logging
logging.basicConfig(filename='sys_and_env_ts.log',
                    level=logging.DEBUG, filemode='w')
logger = logging.getLogger(__name__)

from tulip import transys, spec, synth
import tomatlab
# We label the states using the following picture
#
#     +----+----+----+
#     | X6 | X7 | X8 |
#     +----+----+----+
#     | X3 | X4 | X5 |
#     +----+----+----+
#     | X0 | X1 | X2 |
#     +----+----+----+


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
sys.atomic_propositions.add_from({'goal','home','obsX2', 'obsX4', 'obsX6'})
sys.states.add('X0', ap={'home'})
sys.states.add('X8', ap={'goal'})
sys.states.add('X2', ap={'obsX2'})
sys.states.add('X4', ap={'obsX4'})
sys.states.add('X6', ap={'obsX6'})
# sys.states.add('X7', ap={'obsX7'})


sys2 = transys.FTS()

# Define the states of the system
sys2.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8'])
sys2.states.initial.add('X8')    # start in state X0

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
sys2.atomic_propositions.add_from({'goal','home','obsX2', 'obsX4', 'obsX6'})
sys2.states.add('X8', ap={'home'})
sys2.states.add('X0', ap={'goal'})
sys2.states.add('X2', ap={'obsX2'})
sys2.states.add('X4', ap={'obsX4'})
sys2.states.add('X6', ap={'obsX6'})
"""Park as an env AP
"""
env0 = transys.FTS()
env0.owner = 'env'
env0.states.add_from({'X2', 'X4', 'X6'})
env0.states.initial.add('X2')

env0.atomic_propositions.add_from({'obs2', 'obs4', 'obs6'})
env0.states.add('X2', ap={'obs2'})
env0.states.add('X4', ap={'obs4'})
env0.states.add('X6', ap={'obs6'})
#env0.states.add('d', ap={'obs4'})

env0.transitions.add_from([
    ('X2', 'X4'), ('X4', 'X6'), ('X6', 'X4'), ('X4', 'X2')
    ])
logger.info(env0)

# barely realizable: assumption necessary
env_prog = set()

sys_vars = set()
sys_init = {'home'}
sys_prog = {'goal', 'home'}
sys_safe = {'((obs2) -> X (!obsX2))', '((obs4) -> X (!obsX4))', '((obs6) -> X (!obsX6))'}

print sys

print env0
# # one additional requirement: if in lot,
# # then stay there until park signal is turned off
# sys_safe = {'(X(mem) <-> lot) || (mem && !park)',
#             '((lot && park) -> X(lot))'}
# sys_prog |= {'mem'}

specs = spec.GRSpec(sys_vars=sys_vars, sys_init=sys_init,
                     sys_safety=sys_safe,
                     env_prog=env_prog, sys_prog=sys_prog)
specs.moore = False
specs.qinit = '\A \E'
#ctrl = synth.synthesize('omega', specs, sys=sys, env=env0)
#ctrl.save('sys_and_env_ts0.pdf')
ctrl = synth.synthesize('omega', specs, sys=sys, env=env0)

print specs
print ctrl 

# """Park as an env action
# """
# env1 = transys.FTS()
# env1.owner = 'env'
# env1.states.add('e0')
# env1.states.initial.add('e0')

# env1.env_actions.add_from({'park', 'none'})

# env1.transitions.add('e0', 'e0', env_actions='park')
# env1.transitions.add('e0', 'e0', env_actions='none')
# logger.info(env1)

# env_prog = ['! (env_actions = "park")']
# sys_safe = {'(X(mem) <-> lot) || (mem && ! (env_actions = "park"))',
#             '((lot && (env_actions = "park")) -> X(lot))'}

# specs = spec.GRSpec(sys_vars=sys_vars, sys_init=sys_init,
#                     sys_safety=sys_safe,
#                     env_prog=env_prog, sys_prog=sys_prog)
# specs.moore = False
# specs.qinit = '\A \E'
# ctrl = synth.synthesize('omega', specs, sys=sys, env=env1)
# ctrl.save('sys_and_env_ts1.pdf')
# env1.save('env1.pdf')
# logger.info(ctrl)

# Generate a MATLAB script that generates a Mealy Machine
#tomatlab.export('robot_discrete.mat', ctrl)
