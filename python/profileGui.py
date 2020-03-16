import cProfile
import gui
import pstats

cProfile.run('gui.__main__()','profileMainThread')

p = pstats.Stats('profileMainThread')
p.sort_stats('cumtime')
p.print_stats()

p = pstats.Stats('profileWorkerThread')
p.sort_stats('cumtime')
p.print_stats()
