using Interpolations, Roots

function prelim(params)

# which engine did the student specify?
if params.engine_type==1
	params.revs=(2000. /3.):(2000. /3.):(8000. +2000. /3.)
	params.torque=[98.,124.,146.,195.,220.,225.,220.,248.,254.,244.,230.,209.,186.]
elseif params.engine_type==2
	params.revs=500.:500.:6500.
	params.torque=[156.,176.,215.,299.,319.,345.,332.,312.,312.,299.,280.,234.,169.]
else
	error("Invalid engine type")
end
params.redline=params.revs[end]

params.a=params.wb*(1-params.fwf)
params.b=params.wb*params.fwf

params.Zf=0.5*params.mass*params.g*params.b/(params.a+params.b)
params.Zr=0.5*params.mass*params.g*params.a/(params.a+params.b)

te=CubicSplineInterpolation(params.revs,params.torque)

display("Computing shift speeds...")

if params.drive=="RearWheelDrive"
	params.wtf=1
elseif params.drive=="FrontWheelDrive"
	params.wtf=-1
else
	params.wtf=0
end

params.launchrpm=0.5*params.redline

if length(params.e)==5
	params.eff=0.95
else
	params.eff=0.90
end

params.vmax=0.95*(params.redline*pi/30*(params.rad)/params.ex)./params.e #find redline speeds

display("Speeds in each gear at redline (5% slip) [kph]:")
display(params.vmax*3.6)
display("Speeds range of each gear [kph]:")
display(diff(params.vmax*3.6))

for i=1:length(params.e)-1
	at_bs=params.torque[end]*params.e[i] # axle torque before shift at redline
	at_as=te(params.revs[end]*params.e[i+1]/params.e[i])*params.e[i+1] # axle torque after shift at new engine speed

	if at_as > at_bs
		display("Short shift needed.  Calculating shift speed..."); # change shift speed
		dat(ws) = te(ws)*params.e[i] - te(ws*params.e[i+1]/params.e[i])*params.e[i+1]
		shift_rpm=find_zero(dat,params.revs[end-1])
		params.vmax[i]=shift_rpm*pi/30*params.rad/params.ex/params.e[i]/1.05
	end
end

display("Shift speeds for maximum acceleration [kph]:")
display(params.vmax*3.6)
display("Speeds range of each gear [kph]:")
display(diff(params.vmax*3.6))

w=params.revs[1]:10.0:params.redline
temp=zeros(length(w),2*length(params.e))  # empty space

for n=1:length(params.e)
	u=w*2pi*60*params.rad/1.05/params.ex/params.e[n]/1000  # find speed for each engine speed
 	xt=te.(w)*params.e[n]*params.ex*params.eff/params.rad  # find traction for each engine speed
	temp[:,2n-1]=u  #stor it
 	temp[:,2n]=xt
end

plot(temp[:,1],temp[:,2],label="1",size=(900,600));
for n=2:length(params.e)-1
	plot!(temp[:,2*n-1],temp[:,2*n],label="$n");
end
p=plot!(temp[:,2*length(params.e)-1],temp[:,2*length(params.e)],xlabel="Speed [km/h]",ylabel="Traction force [N]",label="$(length(params.e))");

p
end
