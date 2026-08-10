import casadi as ca

K = 1000

x = ca.SX.sym("x",K)

e = x[0]
for i in range(K):
    e = ca.sin(e)*x[i]

y = e

print(ca.jacobian(y,x).shape)

print(ca.n_nodes(y))
print(ca.n_nodes(ca.jacobian(y,x)))

print(ca.n_nodes(ca.jacobian(y,x,{"helper_options" : {"enable_reverse":False}})))


x = ca.SX.sym("x")
r = ca.SX.sym("r",K)

g = [x]
for i in range(K):
    g.append(ca.sin(r[i]*g[-1]))
y = ca.vcat(g)

print(y.numel())

print(ca.jacobian(y,x).sparsity())

print(ca.n_nodes(y))

print(ca.n_nodes(ca.jacobian(y,x)))

print(ca.n_nodes(ca.jacobian(y,x,{"helper_options" : {"enable_forward":False}})))

