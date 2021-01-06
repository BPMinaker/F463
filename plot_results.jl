using Dates, Plots
plotly()

function plot_results(sol,yout,p,group_ID,params;disp=true)

println("Ran simulation. Plotting results...")

out="results"
~isdir(out) && (mkdir(out))

tmstr="$(group_ID[1])_$(group_ID[2])"
out=joinpath("results",tmstr)
~isdir(out) && (mkdir(out))

# combine the results into a matrix
res=hcat(sol.u...)
res2=hcat(yout...)

# add the static height of the cg to the vertical displacement
res[3,:].+=params.hg

savefig(p,joinpath(out,"perf.html"))
disp && display(p)

plot(params.course[:,1],params.course[:,2],label="Target path",aspect_ratio=:equal,size=(900,600))
p=plot!(res[1,:],res[2,:],xlabel="x [m]",ylabel="y [m]",label="Path")
savefig(p,joinpath(out,"path.html"))
disp && display(p)

p=plot(sol.t,res[21,:],xlabel="Time [s]",ylabel="Distance [m]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"distance.html"))
disp && display(p)

p=plot(sol.t,180/pi*res[6,:],xlabel="Time [s]",ylabel="Yaw angle [deg]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"yaw_angle.html"))
disp && display(p)

p=plot(sol.t,3.6*[res[7,:] res2[7,:]],xlabel="Time [s]",ylabel="Forward, Target speeds [km/h]",label=["Speed" "Target"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"speed.html"))
disp && display(p)

p=plot(sol.t,res[13:16,:]',xlabel="Time [s]",ylabel="Wheel speeds [rad/s]",label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"wheel_speed.html"))
disp && display(p)

p=plot(sol.t,res2[13,:],xlabel="Time [s]",ylabel="Engine speed [rpm]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"rpm.html"))
disp && display(p)

p=plot(sol.t,res2[11:12,:]',xlabel="Time [s]",ylabel="Throttle/Brake",label=["Throttle" "Brake"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"throttle.html"))
disp && display(p)

p=plot(sol.t,res2[14,:],xlabel="Time [s]",ylabel="Gear",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"gear.html"))
disp && display(p)

p=plot(sol.t,180/pi*res[4:5,:]',xlabel="Time [s]",ylabel="Roll, pitch angles [deg]",label=["Roll" "Pitch"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"roll.html"))
disp && display(p)

p=plot(sol.t,res[3,:],xlabel="Time [s]",ylabel="Bounce [m]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"bounce.html"))
disp && display(p)

p=plot(sol.t,180/pi*atan.(res[8,:],res[7,:]),xlabel="Time [s]",ylabel="Body slip angle [deg]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"body_slip.html"))
disp && display(p)

p=plot(sol.t,res[12,:],xlabel="Time [s]",ylabel="Yaw rate [rad/s]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"yaw_rate.html"))
disp && display(p)

p=plot(sol.t,(res2[2,:].+res[7,:].*res[12,:])/9.81,xlabel="Time [s]",ylabel="Lateral accl'n [g]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"lat_acc.html"))
disp && display(p)

p=plot(sol.t,(res2[1,:].-res[8,:].*res[12,:])/9.81,xlabel="Time [s]",ylabel="Long accl'n [g]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"long_acc.html"))
disp && display(p)

p=plot(sol.t,res2[8,:],xlabel="Time [s]",ylabel="Path error [m]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"path_err.html"))
disp && display(p)

p=plot(sol.t,180/pi*res2[9,:],xlabel="Time [s]",ylabel="Heading error [deg]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"heading_err.html"))
disp && display(p)

p=plot(sol.t,180/pi*res2[10,:],xlabel="Time [s]",ylabel="Steer angle [deg]",label="",xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"steer.html"))
disp && display(p)

p=plot(sol.t,res2[19:22,:]',xlabel="Time [s]",ylabel="Long forces [N]",label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"long_frc.html"))
disp && display(p)

p=plot(sol.t,res2[23:26,:]',xlabel="Time [s]",ylabel="Lateral forces [N]",label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"lat_frc.html"))
disp && display(p)

p=plot(sol.t,res2[27:30,:]',xlabel="Time [s]",ylabel="Vertical forces [N]",ylims=(0,Inf),label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"vert_frc.html"))
disp && display(p)

p=plot(sol.t,([-1,-1,1,1].*res[17:20,:])',xlabel="Time [s]",ylabel="Jacking forces [N]",label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"jack_frc.html"))
disp && display(p)

p=plot(sol.t,res2[15:18,:]',xlabel="Time [s]",ylabel="Camber [deg]",label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"camber.html"))
disp && display(p)

p=plot(sol.t,res2[31:34,:]',xlabel="Time [s]",ylabel="Tire slip",ylims=(0,0.5),label=["LF" "LR" "RF" "RR"],xticks=0:5:sol.t[end],size=(900,600))
savefig(p,joinpath(out,"tire_slip.html"))
disp && display(p)

writedlm(joinpath(out,"time_history.txt"),[ res[1:6,:]' sol.t])

out

end
