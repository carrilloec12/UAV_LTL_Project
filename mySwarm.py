#!/usr/bin/env python
"""

    UAV motion planning with moving obstacles.

    Author: Shih-Han Hung
    Date: April 27, 2017
    
    Feature: 
        Motion planning of multiple UAVs in the environment of fixed obstacles.

    Log:
        Support multiple UAV with fixed obstacles (April 22, 2017)
        Support one UAV with fixed obstacles (April 7, 2017)

"""


import random
import logging
from tulip import transys, spec, synth
from pprint import pprint

""" Number of robots """
nobj = 2

""" Dimension """
length = 2
width = 2
height = 2
nstates = length * width * height

""" Location of homes """
home = [[0,0,0], [2,0,0]]

""" Location of lots """
lot = [[2,0,0], [0,2,0]]

""" Location of fixed obstacles """
obs = [[1,1,0]]

""" Initial location of robots"""
init = home[:nobj]


""" represent state s by string """
def st(s):
    return ''.join(['X'+'n'.join([str(x) for x in r]) for r in s])
    #  return pos(s[0])

""" output = grid ^ nobj """
def gen_states(grid, n):
    if n <= 0:
        return [[]]
    elif n == 1:
        return [ [r] for r in grid]
    else:
        return [ s + [r] for r in grid for s in gen_states(grid, n-1)]

""" generate vectors in [m]^n """
def gen_index(m, n):
    if n <= 0:
        return [[]]
    elif n == 1:
        return [ [r] for r in range(m) ]
    else:
        return [ s + [r] for r in range(m) for s in gen_index(m, n-1) ]

def index_to_state(grid, i):
    return [ grid[r] for r in i]

""" grid = [[0,0], [0,1], ...] """
grid = [[x,y,z] for z in range(height) for y in range(width) for x in range(length)]
#  print(grid)

""" state = grid x grid x ... """
#  states = gen_states(grid, nobj)
#  print(states)
idx = gen_index(nstates, nobj)
#  print(idx)
#  print([ index_to_state(grid, i) for i in idx ])

sys = transys.FTS()
#  sys.states.add_from([ st(s) for s in states])
for i in idx:
    s = index_to_state(grid, i)
    sys.states.add( st(s) )
#  sys.states.add_from([ st([grid[x] for x in i]) in i in idx])
sys.states.initial.add(st(init))

#  world = [gen_states(grid, k) for k in range(nobj)]
#  print("World generated...")
wld = [ gen_index(nstates, k) for k in range(nobj)]
#  print(wld)

for k in range(nobj):  
    for r in grid:
        i = r[0]
        j = r[1]
        m = r[2]

        ''' Construct transitions '''
        for p1 in wld[k]:
            for p2 in wld[nobj-k-1]:
                s1 = index_to_state(grid, p1)
                s2 = index_to_state(grid, p2)
                s = s1 + [r] + s2
                if i + 1 < length:
                    ns = s1 + [[i+1,j,m]] + s2
                    sys.transitions.add_comb( { st(s) }, { st(ns) } )
                if i - 1 >= 0:
                    ns = s1 + [[i-1,j,m]] + s2
                    sys.transitions.add_comb( { st(s) }, { st(ns) } )
                if j + 1 < width:
                    ns = s1 + [[i,j+1,m]] + s2
                    sys.transitions.add_comb( { st(s) }, { st(ns) } )
                if j - 1 >= 0:
                    ns = s1 + [[i,j-1,m]] + s2
                    sys.transitions.add_comb( { st(s) }, { st(ns) } )
                if m + 1 < height:
                    ns = s1 + [[i,j,m+1]] + s2
                    sys.transitions.add_comb( { st(s) }, { st(ns) } )
                if m - 1 >= 0:
                    ns = s1 + [[i,j,m-1]] + s2
                    sys.transitions.add_comb( { st(s) }, { st(ns) } )

print("Transitions connected...")

atomic = dict()

''' Place home, lot location '''
for k in range(nobj):

    sys.atomic_propositions.add_from({'home'+str(k), 'lot'+str(k)})

    for p1 in wld[k]:
        for p2 in wld[nobj-k-1]:
            s1 = index_to_state(grid, p1)
            s2 = index_to_state(grid, p2)
            shome = s1 + [home[k]] + s2
            if st(shome) not in atomic:
                atomic[st(shome)] = ['home'+str(k)]
            else:
                atomic[st(shome)] += ['home'+str(k)]

            slot = s1 + [lot[k]] + s2
            if st(slot) not in atomic:
                atomic[st(slot)] = ['lot'+str(k)]
            else:
                atomic[st(slot)] += ['lot'+str(k)]
 

""" Label states as obstacles """
sys.atomic_propositions.add_from({'obs'})

''' Inter-robot collision avoidance '''
for i in idx:
    s = index_to_state(grid, i)
    if len(s) != len(set([str(r) for r in s])):
        sts = st(s)
        if sts not in atomic:
            atomic[sts] = ['obs']
        else:
            atomic[sts] += ['obs']

""" Obstacle avoidance """
for i in idx:
    s = index_to_state(grid, i)
    if not set([str(r) for r in s]).isdisjoint([str(r) for r in obs]):
        sts = st(s)
        if sts not in atomic:
            atomic[sts] = ['obs']
        else:
            atomic[sts] += ['obs']


for key in atomic:
    sys.states.add(key, ap=set(atomic[key]))

print("Atomic propositions added...")

del grid, wld, idx, atomic

#  print(sys)
#  sys.plot()
#  xx = raw_input("Pause...")

env_vars = {'park'} # park signal is turned on initially.
env_init = {}
#  env_prog = {'!park'}  # park signal is turned off infinitely often.
#  env_safe = {}
env_prog = {}  # park signal is turned off infinitely often.
env_safe = {'park'}

sys_vars = set(['mem'+str(k) for k in range(nobj)]) # {'mem0'}
#  sys_init = set(['mem'+str(k) for k in range(nobj)]) # {'mem0'}
sys_init = {}
sys_prog = set(['home'+str(k) for k in range(nobj)]) | set(['mem'+str(k) for k in range(nobj)])# {'home0', 'mem0'}

# Equivalent LTL formula: []<> home0 && [](park -> <> lot0)
# For two objects, &&( []<> home[k] && [](park -> <> lot[k]) )
ltl_safe = [ '((X (mem' + str(k)
            + ') <-> lot'+ str(k)  
            + ') || (mem' + str(k)  
            + ' && !park))'         for k in range(nobj) ] + [ '(!obs)' ]

#  print(ltl_safe)
sys_safe = {' && '.join(ltl_safe)}

#  print(sys)

specs = spec.GRSpec( env_vars, sys_vars,
                    env_init, sys_init,
                    env_safe, sys_safe,
                    env_prog, sys_prog )

print("Specs...")

specs.moore = True
specs.qinit = '\E \A'
ctrl = synth.synthesize( 'omega', specs, sys=sys )
assert ctrl is not None, 'unrealizable'

print("Synthesis done...")

cs = 'Sinit'
counter = 1
with open(''.join([ str(i) for i in [nobj, length, width, height, '.out']]), 'w+') as f:
    while counter <= 1000:
        ns, nv = random.choice(ctrl.adj[cs].items())
        pos = [ r.split('n') for r in nv[0]['loc'].split('X')[1:]]
        f.write(str(ns) + "\t" + "\t".join([ "\t".join(r) for r in pos]) + "\t park=" + str(int(nv[0]['park'])) + '\n')
        cs = ns
        counter += 1

#  ctrl.save('plot.png')
if not ctrl.save('plot.png'):
    print(ctrl)

