from __future__ import print_function
import time
from tulip import spec
from tulip import synth
from tulip.transys import machines



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

env_vars = {'obs': range(27)}

sys_vars = {'uav': range(27)}

env_init = {'(obs = 10)'}

env_prog = {'(obs = 16)'}

env_safe = {'((obs = 0) -> (X obs = 1) || (X obs = 3) || (X obs = 9))',
            '((obs = 1) -> (X obs = 0) || (X obs = 2) || (X obs = 4) || (X obs = 10))',
            '((obs = 2) -> (X obs = 1) || (X obs = 5) || (X obs = 11))',
            '((obs = 3) -> (X obs = 0) || (X obs = 4) || (X obs = 6) || (X obs = 12))',
            '((obs = 4) -> (X obs = 1) || (X obs = 3) || (X obs = 5) || (X obs = 7) || (X obs = 13))',
            '((obs = 5) -> (X obs = 2) || (X obs = 4) || (X obs = 8) || (X obs = 14))',
            '((obs = 6) -> (X obs = 3) || (X obs = 7) || (X obs = 15))',
            '((obs = 7) -> (X obs = 4) || (X obs = 6) || (X obs = 8) || (X obs = 16))',
            '((obs = 8) -> (X obs = 5) || (X obs = 7) || (X obs = 17))',
            '((obs = 9) -> (X obs = 0) || (X obs = 10) || (X obs = 12) || (X obs = 18))',
            '((obs = 10) -> (X obs = 1) || (X obs = 9) || (X obs = 11) || (X obs = 13) || (X obs = 19))',
            '((obs = 11) -> (X obs = 2) || (X obs = 10) || (X obs = 14) || (X obs = 20))',
            '((obs = 12) -> X( obs = 3) || X(obs = 9) || X(obs = 13) || X(obs = 15) || X(obs = 21))',
            '((obs = 13) -> (X obs = 4) || (X obs = 10) || (X obs = 12) || (X obs = 14) || (X obs = 16) || (X obs = 22))',
            '((obs = 14) -> (X obs = 5) || (X obs = 11) || (X obs = 13) || (X obs = 17) || (X obs = 23))',
            '((obs = 15) -> (X obs = 6) || (X obs = 12) || (X obs = 16) || (X obs = 24))',
            '((obs = 16) -> (X obs = 7) || (X obs = 13) || (X obs = 15) || (X obs = 17) || (X obs = 25))',
            '((obs = 17) -> (X obs = 8) || (X obs = 14) || (X obs = 16) || (X obs = 26))',
            '((obs = 18) -> (X obs = 9) || (X obs = 19) || (X obs = 21))',
            '((obs = 19) -> (X obs = 10) || (X obs = 18) || (X obs = 20) || (X obs = 22))',
            '((obs = 20) -> (X obs = 11) || (X obs = 19) || (X obs = 23))',
            '((obs = 21) -> (X obs = 12) || (X obs = 18) || (X obs = 22) || (X obs = 24))',
            '((obs = 22) -> (X obs = 13) || (X obs = 19) || (X obs = 21) || (X obs = 23) || (X obs = 25))',
            '((obs = 23) -> (X obs = 14) || (X obs = 20) || (X obs = 22) || (X obs = 26))',
            '((obs = 24) -> (X obs = 15) || (X obs = 21) || (X obs = 25))',
            '((obs = 25) -> (X obs = 16) || (X obs = 22) || (X obs = 24) || (X obs = 26))',
            '((obs = 26) -> (X obs = 17) || (X obs = 23) || (X obs = 25))'
            }

sys_init = {'(uav = 12)'}

sys_prog = {'(uav = 14)'}

sys_safe = {'((uav = 0) -> (X uav = 1) || (X uav = 3) || (X uav = 9))',
            '((uav = 1) -> (X uav = 0) || (X uav = 2) || (X uav = 4) || (X uav = 10))',
            '((uav = 2) -> (X uav = 1) || (X uav = 5) || (X uav = 11))',
            '((uav = 3) -> (X uav = 0) || (X uav = 4) || (X uav = 6) || (X uav = 12))',
            '((uav = 4) -> (X uav = 1) || (X uav = 3) || (X uav = 5) || (X uav = 7) || (X uav = 13))',
            '((uav = 5) -> (X uav = 2) || (X uav = 4) || (X uav = 8) || (X uav = 14))',
            '((uav = 6) -> (X uav = 3) || (X uav = 7) || (X uav = 15))',
            '((uav = 7) -> (X uav = 4) || (X uav = 6) || (X uav = 8) || (X uav = 16))',
            '((uav = 8) -> (X uav = 5) || (X uav = 7) || (X uav = 17))',
            '((uav = 9) -> (X uav = 0) || (X uav = 10) || (X uav = 12) || (X uav = 18))',
            '((uav = 10) -> (X uav = 1) || (X uav = 9) || (X uav = 11) || (X uav = 13) || (X uav = 19))',
            '((uav = 11) -> (X uav = 2) || (X uav = 10) || (X uav = 14) || (X uav = 20))',
            '((uav = 12) -> (X uav = 3) || (X uav = 9) || (X uav = 13) || (X uav = 15) || (X uav = 21))',
            '((uav = 13) -> (X uav = 4) || (X uav = 10) || (X uav = 12) || (X uav = 14) || (X uav = 16) || (X uav = 22))',
            '((uav = 14) -> (X uav = 5) || (X uav = 11) || (X uav = 13) || (X uav = 17) || (X uav = 23))',
            '((uav = 15) -> (X uav = 6) || (X uav = 12) || (X uav = 16) || (X uav = 24))',
            '((uav = 16) -> (X uav = 7) || (X uav = 13) || (X uav = 15) || (X uav = 17) || (X uav = 25))',
            '((uav = 17) -> (X uav = 8) || (X uav = 14) || (X uav = 16) || (X uav = 26))',
            '((uav = 18) -> (X uav = 9) || (X uav = 19) || (X uav = 21))',
            '((uav = 19) -> (X uav = 10) || (X uav = 18) || (X uav = 20) || (X uav = 22))',
            '((uav = 20) -> (X uav = 11) || (X uav = 19) || (X uav = 23))',
            '((uav = 21) -> (X uav = 12) || (X uav = 18) || (X uav = 22) || (X uav = 24))',
            '((uav = 22) -> (X uav = 13) || (X uav = 19) || (X uav = 21) || (X uav = 23) || (X uav = 25))',
            '((uav = 23) -> (X uav = 14) || (X uav = 20) || (X uav = 22) || (X uav = 26))',
            '((uav = 24) -> (X uav = 15) || (X uav = 21) || (X uav = 25))',
            '((uav = 25) -> (X uav = 16) || (X uav = 22) || (X uav = 24) || (X uav = 26))',
            '((uav = 26) -> (X uav = 17) || (X uav = 23) || (X uav = 25))',
            '(uav != obs)'
            }

'''
disc_props = dict()
#UAV
disc_props['U0'] = '(uav = 0)'
disc_props['U1'] = '(uav = 1)'
disc_props['U2'] = '(uav = 2)'
disc_props['U3'] = '(uav = 3)'
disc_props['U4'] = '(uav = 4)'
disc_props['U5'] = '(uav = 5)'
disc_props['U6'] = '(uav = 6)'
disc_props['U7'] = '(uav = 7)'
disc_props['U8'] = '(uav = 8)'
disc_props['U9'] = '(uav = 9)'
disc_props['U10'] = '(uav = 10)'
disc_props['U11'] = '(uav = 11)'
disc_props['U12'] = '(uav = 12)'
disc_props['U13'] = '(uav = 13)'
disc_props['U14'] = '(uav = 14)'
disc_props['U15'] = '(uav = 15)'
disc_props['U16'] = '(uav = 16)'
disc_props['U17'] = '(uav = 17)'
disc_props['U18'] = '(uav = 18)'
disc_props['U19'] = '(uav = 19)'
disc_props['U20'] = '(uav = 20)'
disc_props['U21'] = '(uav = 21)'
disc_props['U22'] = '(uav = 22)'
disc_props['U23'] = '(uav = 23)'
disc_props['U24'] = '(uav = 24)'
disc_props['U25'] = '(uav = 25)'
disc_props['U26'] = '(uav = 26)'
# Obstacle
disc_props['B0'] = '(obs = 0)'
disc_props['B1'] = '(obs = 1)'
disc_props['B2'] = '(obs = 2)'
disc_props['B3'] = '(obs = 3)'
disc_props['B4'] = '(obs = 4)'
disc_props['B5'] = '(obs = 5)'
disc_props['B6'] = '(obs = 6)'
disc_props['B7'] = '(obs = 7)'
disc_props['B8'] = '(obs = 8)'
disc_props['B9'] = '(obs = 9)'
disc_props['B10'] = '(obs = 10)'
disc_props['B11'] = '(obs = 11)'
disc_props['B12'] = '(obs = 12)'
disc_props['B13'] = '(obs = 13)'
disc_props['B14'] = '(obs = 14)'
disc_props['B15'] = '(obs = 15)'
disc_props['B16'] = '(obs = 16)'
disc_props['B17'] = '(obs = 17)'
disc_props['B18'] = '(obs = 18)'
disc_props['B19'] = '(obs = 19)'
disc_props['B20'] = '(obs = 20)'
disc_props['B21'] = '(obs = 21)'
disc_props['B22'] = '(obs = 22)'
disc_props['B23'] = '(obs = 23)'
disc_props['B24'] = '(obs = 24)'
disc_props['B25'] = '(obs = 25)'
disc_props['B26'] = '(obs = 26)'
for index in range(len(sys_safe)):
    for subformula in disc_props:
        sys_safe[index] = sys_safe[index].replace(subformula, disc_props[subformula])
'''

specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

specs.moore = False
specs.qinit = r'\A \E'
strategy = synth.synthesize('omega', specs)
assert strategy is not None, 'unrealizable'
if not strategy.save('UAV_Transition.png'):
    print(strategy)