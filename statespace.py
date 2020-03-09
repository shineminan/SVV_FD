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

	q[0] = [-CA.CXu, -CA.CXa, -CA.CZ0, -CA.CXq]
	q[1] = [-CA.CZu, -CA.CZa, CA.CX0, -(CA.CZq+2*CA.muc)]
	q[2, -1] = -1
	q[3] = [-CA.Cmu, -CA.Cma, 0, -CA.Cmq]

	r = np.array([[-CA.CXde], [-CA.CZde],[0],[-CA.Cmde]])
	A = np.linalg.inv(p).dot(q)
	B = np.linalg.inv(p).dot(r)
	return A, B, r, q

def asymmetric_motion_solver():
	V = CA.V0
	p = np.zeros((4, 4))
	q = np.zeros((4, 4))
	p[0,0] = (CA.CYbdot-2*CA.mub)*CA.b/V
	p[1,1] = -CA.b/2/V
	p[2,2:] = [-4*CA.mub*CA.KX2*CA.b/V,4*CA.mub*CA.KXZ*CA.b/V]
	p[3] = [CA.Cnbdot*CA.b/V,0,4*CA.mub*CA.KXZ*CA.b/V,-4*CA.mub*CA.KZ2*CA.b/V]

	q[0] = [-CA.CYb,-CA.CL,-CA.CYp,-(CA.CYr-4*CA.mub)]
	q[1,2] = -1
	q[2] = [-CA.Clb,0,-CA.Clp,-CA.Clr]
	q[3] = [-CA.Cnb,0,-CA.Cnp,-CA.Cnr]

	r = np.array([[-CA.CYda,-CA.CYdr], [0,0], [-CA.Clda,-CA.Cldr], [-CA.Cnda,-CA.Cndr]])
	A = np.linalg.inv(p).dot(q)
	B = np.linalg.inv(p).dot(r)
	return A, B, r, q


# ws,vs = np.linalg.eig(symmetric_motion_solver()[0])
# for item in ws:
# 	print(item)
#
# print(vs)
# wa,va = np.linalg.eig(asymmetric_motion_solver()[0])

def period(a):  # short and long
	result = []
	if a[0].real() < a[2].real():
		result.append((a[0].real(), a[0].imag()))
		result.append((a[2].real(), a[2].imag()))
	else:
		result.append((a[2].real(), a[2].imag()))
		result.append((a[0].real(), a[0].imag()))
	return result


def motion(char = None, mode = None):
	if char == "symmetric":
		A,B,r,q = symmetric_motion_solver()
		if mode == "short":
			a, b = period(np.linalg.eig(A))[0]
		elif mode == "long":
			a, b = period(np.linalg.eig(A))[1]
	elif char == "asymmetric":
		A, B, r, q = asymmetric_motion_solver()
		if mode == "short":
			a, b = period(np.linalg.eig(A))[0]
		elif mode == "long":
			a, b = period(np.linalg.eig(A))[1]
	thalf = np.log(CA.c/2)/a/CA.V0
	p = 2*np.pi/CA.b*CA.c/CA.V0
	return thalf, p, complex(a,b)







