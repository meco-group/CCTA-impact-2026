import casadi as ca

x = ca.MX.sym('x') 
u = ca.MX.sym('u')

F = ca.Function('F',[x,u],[x**2+u])

N = 4
 
opti = ca.Opti()

U = opti.variable(N)

x = 2
X = [x]


for k in range(N):
  x = F(x,U[k])
  X.append(x)

X = ca.hcat(X)

opti.subject_to(X[1:-1]>=0)
opti.subject_to(X[-1]==3)

opti.minimize(ca.sumsqr(U)+ca.sumsqr(X))

opti.solver('ipopt')
sol = opti.solve()

print(sol.value(U)) # should be [-2.7038;-0.5430;0.2613;0.5840]
