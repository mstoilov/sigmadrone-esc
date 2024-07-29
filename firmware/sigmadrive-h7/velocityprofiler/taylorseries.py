import matplotlib.pyplot as pp
import numpy as np

def taylor_cos(t):
	t2 = t * t
	t4 = t2 * t2
	t6 = t2 * t4
	t8 = t2 * t6	
	cos_t = 1.0 - t2/(1*2)# + t4/(1*2*3*4) - t6/(1*2*3*4*5*6) + t8/(1*2*3*4*5*6*7*8)
	return cos_t

def taylor_sin(t):
	t2 = t * t
	t4 = t2 * t2
	t6 = t2 * t4
	t8 = t2 * t6	
	sin_t = t - t*t2/(1*2*3)# + t*t4/(1*2*3*4*5) - t*t6/(1*2*3*4*5*6*7) + t*t8/(1*2*3*4*5*6*7*8*9)
	return sin_t


x = np.arange(-np.pi, np.pi, 0.01)
y = np.cos(x)

pp.plot(x,np.cos(x), color='blue')
pp.plot(x,np.sin(x), color='blue')
pp.plot(x, taylor_cos(x), color='orange')
pp.plot(x, taylor_sin(x), color='orange')

pp.show()
