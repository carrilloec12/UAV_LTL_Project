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


# UAV 1 
sys = transys.FTS()
sys.owner = 'sys'

# Define the states of the system
sys.states.add_from(['X0','X1','X2','X3','X4','X5','X6','X7','X8','X9','X10','X11','X12','X13','X14','X15','X16'])
sys.states.initial.add('X0')    # start in state X0

# Define the allowable transitions
#! TODO (IF): can arguments be a singleton instead of a list?
#! TODO (IF): can we use lists instead of sets?
#!   * use optional flag to allow list as label
sys.transitions.add_comb({'X0'}, {'X1','X4'})
sys.transitions.add_comb({'X1'}, {'X5','X0','X2'})
sys.transitions.add_comb({'X2'}, {'X6','X1','X3'})
sys.transitions.add_comb({'X3'}, {'X2','X7'})
sys.transitions.add_comb({'X4'}, {'X5','X0','X8'})
sys.transitions.add_comb({'X5'}, {'X1','X6','X4','X9'})
sys.transitions.add_comb({'X6'}, {'X7','X2','X5','X10'})
sys.transitions.add_comb({'X7'}, {'X3','X6','X11'})
sys.transitions.add_comb({'X8'}, {'X12','X4','X9'})
sys.transitions.add_comb({'X9'}, {'X5','X13','X10','X8'})
sys.transitions.add_comb({'X10'}, {'X14','X6','X11','X9'})
sys.transitions.add_comb({'X11'}, {'X15','X7','X10'})
sys.transitions.add_comb({'X12'}, {'X13','X8'})
sys.transitions.add_comb({'X13'}, {'X9','X12','X14'})
sys.transitions.add_comb({'X14'}, {'X10','X13','X15'})
sys.transitions.add_comb({'X15'}, {'X14','X11'})
# @system_dynamics_section_end@

# @system_labels_section@
# Add atomic propositions to the states
sys.atomic_propositions.add_from({'goal','obsX3','home','obsX12','obsX6', 'obsX9'})
sys.states.add('X0', ap={'home'})
sys.states.add('X15', ap={'goal'})
sys.states.add('X3', ap={'obsX3'})
sys.states.add('X6', ap={'obsX6'})
sys.states.add('X9', ap={'obsX9'})
sys.states.add('X12', ap={'obsX12'})
"""Park as an env AP
"""
env0 = transys.FTS()
env0.owner = 'env'
env0.states.add_from({'X3', 'X6', 'X9', 'X12'})
env0.states.initial.add('X3')

env0.atomic_propositions.add_from({'obs3', 'obs6', 'obs9', 'obs12'})
env0.states.add('X3', ap={'obs3'})
env0.states.add('X6', ap={'obs6'})
env0.states.add('X9', ap={'obs9'})
env0.states.add('X12', ap={'obs12'})

env0.transitions.add_from([
    ('X3', 'X6'), ('X6', 'X9'), ('X9', 'X12'), ('X12', 'X9'), ('X9', 'X6'), ('X6', 'X3')
])
logger.info(env0)

# barely realizable: assumption necessary
env_prog = set()

sys_vars = set()
sys_init = {'home'}
sys_prog = {'goal', 'home'}
sys_safe = {'((obs3) -> X (!obsX3))', '((obs6) -> X (!obsX6))', '((obs9) -> X (!obsX9))', '((obs12) -> X (!obsX12))'}

#print sys

#print env0
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
ctrl = synth.synthesize('omega', specs, sys=sys, env=env0)
#ctrl.save('sys_and_env_ts0.pdf')

#print specs
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
