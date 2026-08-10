import casadi as ca

N = 1000

x = ca.SX.sym("x",N)

e = x[0]
for i in range(N):
    e = e*ca.sin(e)

y = e

print(ca.n_nodes(y))

print(ca.n_nodes(ca.jacobian(y,x)))

