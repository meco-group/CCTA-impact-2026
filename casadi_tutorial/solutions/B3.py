import numpy as np
import casadi as ca

# In this file we create a CasADi Function that fits Alvik distance measurements

angles = ca.vertcat(-45,-22,0,22,45)  # There are 5 sensor in front with different angles
angled_rad = angles/180*np.pi

# We use an optimization problem
# (yes it's just linear least-squares)
opti = ca.Opti('conic')

d = opti.parameter(5) # Measured distances in cm (not known at construction time obviously); 5-by-1

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

opti.solver("qrqp")

# Convert to regular CasADi Function
fitter = opti.to_function('fitter',[d],[a,b],["d"],["a","b"])

print(fitter) # fitter:(d[5])->(a,b) MXFunction

# Generate code
fitter.generate('fitter.c',{"with_header":True})

# Small experiment
print(fitter(ca.vertcat(6.7,6.2,6,5.9,5.7)))
