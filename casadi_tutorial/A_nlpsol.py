import casadi as ca
from numpy import inf

x = ca.MX.sym('x') 
u = ca.MX.sym('u')

F = ca.Function('F',[x,u],[x**2+u])

U = ca.MX.sym('U',4)

x0 = 2
x1 = F(x0,U[0])
x2 = F(x1,U[1])
x3 = F(x2,U[2])
x4 = F(x3,U[3])


nlp = {}
nlp['x'] = U

nlp['g'] = ca.vertcat(x1, x2, x3,x4)
lbg = ca.vertcat(0,  0,  0,  3)
ubg = ca.vertcat(inf,inf,inf,3)

X = ca.vertcat(x0,x1,x2,x3,x4)

nlp['f'] = ca.sumsqr(U)+ca.sumsqr(X)

solver = ca.nlpsol('solver','ipopt',nlp)

print(solver(lbg=lbg,ubg=ubg))

print(solver)
print(type(solver))

