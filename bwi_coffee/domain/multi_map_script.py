#!/usr/bin/python
import os
import commands
import sys

os.system('touch combined.asp')
os.system('rm combined.asp')
os.system('cat multi_map_coffee.asp  query.asp  multi_map_facts.asp multi_map_initial.asp > combined.asp')
for i in range(100):
  s = ''
  s = commands.getoutput(\
  'clingo ' + 'combined.asp -c n=' + str(i))

  if s.find('UNSATISFIABLE') == -1:
    print(s)
    print('SATISFIABLE: n=' + str(i))
    break
  sys.stdout.write('.')
  sys.stdout.flush()

os.system('rm combined.asp')
