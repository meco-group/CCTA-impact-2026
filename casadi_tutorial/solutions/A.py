import casadi as ca

x = ca.MX.sym('x') 
u = ca.MX.sym('u')

F = ca.Function('F',[x,u],[x**2+u])

opti = ca.Opti()

U = opti.variable(4)

x0 = 2
x1 = F(x0,U[0])
x2 = F(x1,U[1])
x3 = F(x2,U[2])
x4 = F(x3,U[3])

opti.subject_to(x1>=0)
opti.subject_to(x2>=0)
opti.subject_to(x3>=0)
opti.subject_to(x4==3)

X = ca.vertcat(x0,x1,x2,x3,x4)

opti.minimize(ca.sumsqr(U)+ca.sumsqr(X))

opti.solver('ipopt')
sol = opti.solve()

print(sol.value(opti.f))
