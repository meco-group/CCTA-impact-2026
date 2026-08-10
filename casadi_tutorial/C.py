import casadi as ca

N = 1000

x = ca.SX.sym("x",N)

e = x[0]
for i in range(N):
    e = e*ca.sin(e)

y = e

print(ca.jacobian(y,x).shape)

print(ca.n_nodes(y))


print(ca.n_nodes(ca.jacobian(y,x)))



x = ca.SX.sym("x")
r = ca.SX.sym("r",1000)

g = ca.sin(r*x)



