from optimo import execute
from DFSAllPath import UsefullEdgesDFS
import time

for i in range(600-10):
    start_time = time.time()
    execute()
    end_time = time.time()
    print(f'time case {i}: {end_time - start_time}')