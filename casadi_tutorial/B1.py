import numpy as np
import casadi as ca

# In this file we create a CasADi Function that fits Alvik distance measurements

angles = ca.vertcat(-45,-22,0,22,45)  # There are 5 sensor in front with different angles
angled_rad = angles/180*np.pi

# We use an optimization problem
opti = ca.Opti()

d = ca.vertcat(6.7,6.2,6,5.9,5.7)

# Cartesian positions of measured artifacts
X = d*np.cos(angled_rad) # 5-by-1
Y = d*np.sin(angled_rad) # 5-by-1

# Line parametrization
a = opti.variable() # Orientation
b = opti.variable() # Offset/position

# Residual
res = X-(a*Y+b) # 5-by-1

# Least-squares
opti.minimize(ca.sumsqr(res))

opti.solver("ipopt")

sol = opti.solve()

print(sol.value(a))
