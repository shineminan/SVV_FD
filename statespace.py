import numpy as np
import Cit_par as CA

def symmetric_motion_solver():
	V = CA.V0
	p = np.zeros((4, 4))
	q = np.zeros((4, 4))
	p[0, 0] = -2*CA.muc*CA.c/V
	p[1, 1] = (CA.CZadot-2*CA.muc)*CA.c/V
	p[2, 2] = -CA.c/V
	p[3, 1] = CA.Cmadot*CA.c/V
	p[3, 3] = -2*CA.muc*CA.KY2*CA.c/V
	print(p)

	q[0] = [-CA.CXu, -CA.CXa, -CA.CZ0, -CA.CXq]
	q[1] = [-CA.CZu, -CA.CZa, CA.CX0, -(CA.CZq+2*CA.muc)]
	q[2, -1] = -1
	q[3] = [-CA.Cmu, -CA.Cma, 0, -CA.Cmq]
	print(q)

symmetric_motion_solver()






