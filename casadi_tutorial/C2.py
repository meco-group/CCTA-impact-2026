import casadi as ca

K = 1000

x = ca.SX.sym("x")
r = ca.SX.sym("r",K)

g = [x]
for i in range(K):
    g.append(ca.sin(r[i]*g[-1]))
y = ca.vcat(g)

print(ca.n_nodes(y))

print(ca.n_nodes(ca.jacobian(y,x)))

