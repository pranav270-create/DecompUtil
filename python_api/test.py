import py_seed_decomp
import numpy as np

obs = np.random.randn(100)
obs = obs.reshape((50, 2))
ret = py_seed_decomp.seed_decomp(0, 0, obs)
ret = np.array(ret)
A = ret[:, :2]
b = ret[:, 2]
print(A)
print(b)
