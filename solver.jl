
using OrdinaryDiffEq
#using DifferentialEquations

# define function to decide when to stop simulation, stops when value crosses zero
function dist_to_end(z,t,integrator)

# if the distance travelled is too short, stop when travelling backward, when close to end, use x coordinate (finish line)
if z[21]<2200
	value=-z[7]
else
	value=z[1]
end

value

end

# function to launch ode solver
function solver(params)

println("Gathered data. Simulating...")

# set max end time and plot interval
tend=165
tspan=(0.,tend)

# set initial conditions, start at 1 m/s instead of 0 to avoid tire model issues, find matching wheel speeds
x0=zeros(23,1)
x0[7]=1.0
x0[13:16]=x0[7]/params.rad*[1,1,1,1]

# define function that contains the equations to solve, and solve them, but don't return extra data until after solution complete
cb=ContinuousCallback(dist_to_end,terminate!,interp_points=1000)
prob=ODEProblem(eqn_of_motion,x0,tspan,params)
#sol=solve(prob,Tsit5(),saveat=0.1,dt=1.e-3,dtmax=0.01,callback=cb,progress=true)
# alternate stiff solver for challenging parameters
# need to switch library to DifferentialEquations
sol=solve(prob,Tsit5(),saveat=0.1,dt=1.e-3,dtmax=0.01,abstol=1.e-4,callback=cb,progress=true)
#,alg_hint=:stiff)
# recalculate extra derivative data (accelerations)
# define empty vector of vectors, call ode function
yout=[Vector{Float64}(undef,34) for i in sol.t]
for i=1:length(sol.t)
	eqn_of_motion(yout[i],sol.u[i],params,sol.t[i],flag=true)
end

sol,yout

end
