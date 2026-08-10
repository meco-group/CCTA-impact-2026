import casadi as ca

x = ca.MX.sym('x') 
u = ca.MX.sym('u')

F = ca.Function('F',[x,u],[x**2+u])

N = 4

opti = ca.Opti()

X = opti.variable(N+1)
U = opti.variable(N)

for k in range(N):
  opti.subject_to(F(X[k],U[k])==X[k+1])

opti.subject_to(X[0]==2)
opti.subject_to(X[1:-1]>=0)
opti.subject_to(X[-1]==3)

opti.minimize(ca.sumsqr(U)+ca.sumsqr(X))

opti.solver('ipopt')
sol = opti.solve()

print(sol.value(U)) # should be [-2.7038;-0.5430;0.2613;0.5840]
