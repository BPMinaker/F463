
using Interpolations

function skew(vct)
	[0. -vct[3] vct[2]; vct[3] 0. -vct[1]; -vct[2] vct[1] 0.]
end

function eqn_of_motion(zdot,z,params,t;flag=false)
# println(t)

pos=z[1:3] # unload reference frame location and velocities from state
roll=z[4]
pitch=z[5]
yaw=z[6]

vel=z[7:9] # linear velocity
ang_vel=z[10:12] # angular velocity
wheel_angvel=z[13:16]
jk_frc=z[17:20]

s=z[21] # distance travelled
delta=z[22]
throttle=z[23]

# kinematics
s_roll=sin(roll)
c_roll=cos(roll)
s_pitch=sin(pitch)
c_pitch=cos(pitch)
s_yaw=sin(yaw)
c_yaw=cos(yaw)

t_pitch=tan(pitch)

rot_mtx=transpose([c_pitch*c_yaw c_pitch*s_yaw -s_pitch;
	s_roll*s_pitch*c_yaw-c_roll*s_yaw s_roll*s_pitch*s_yaw+c_roll*c_yaw s_roll*c_pitch;
	c_roll*s_pitch*c_yaw+s_roll*s_yaw c_roll*s_pitch*s_yaw-s_roll*c_yaw c_roll*c_pitch])

glbl_vel=rot_mtx*vel # find global velocity of reference frame, integrate this for global location

roll_rate=ang_vel[1]+s_roll*t_pitch*ang_vel[2]+c_roll*t_pitch*ang_vel[3]
pitch_rate=c_roll*ang_vel[2]-s_roll*ang_vel[3]
yaw_rate=s_roll/c_pitch*ang_vel[2]+c_roll/c_pitch*ang_vel[3]

euler_rates=[roll_rate,pitch_rate,yaw_rate]

# using location, angle of reference frame, find offset from track, use s distance to find approximate closest point, speed for lookahead distance
path_error,heading_error,u_ref,curv=track_offset(pos[1:2],yaw,s,vel[1],params)


# driver model
k=[10,10,6,2,0.8,0.16,0.04,0.01] # weights of preview points
k/=sum(k) # normalized
path_error=k' *path_error
heading_error=k' *heading_error

# TRY this sometime (Sijie driver mdel)
#K = (1.55*1.55 *path_error + 2 * 0.11 * 1.55 * vel(1)*sin(heading_error))/vel(1)^2;
#Kc = ang_vel(3)/(vel(1));
#err= 15*(curv+K-Kc)+ (curv+K)*2.6;
#delta_dot=10*(err-delta);

# calculate the steering angle based on road curvature, plus error
err=params.wb*(curv)+1.1*heading_error+0.1*path_error
# find delta by integrating, effectively adds low pass filter
delta_dot=20. *(err-delta)

# if steer is very large and increasing
if abs(delta)>1 && delta*delta_dot > 0
	delta_dot=0
end

# note camber is actually wheel roll, same sign convention left and right
camber=roll*[params.df,params.dr,params.df,params.dr]+2*[params.df-1,params.dr-1,1-params.df,1-params.dr].*(pos[3].+pitch*[-params.a,params.b,-params.a,params.b])/params.t
camber*=180/pi

sdelta=sin(delta)
cdelta=cos(delta)

# compute the velocity of the tire centre
wheel_vel_X=vel[1].+ang_vel[3]*params.t/2*[-1,-1,1,1] # in vehicle X
wheel_vel_Y=vel[2].+ang_vel[3]*[params.a,-params.b,params.a,-params.b] # in vehicle Y

# transform to tire coordinate system
# start with copy, rear wheels ([2 4]) are same
wheel_vel_x=copy(wheel_vel_X)
wheel_vel_y=copy(wheel_vel_Y)

# front wheel calc
wheel_vel_x[[1,3]]=wheel_vel_X[[1,3]]*cdelta+wheel_vel_Y[[1,3]]*sdelta # in tire x
wheel_vel_y[[1,3]]=wheel_vel_Y[[1,3]]*cdelta-wheel_vel_X[[1,3]]*sdelta # in tire y

# subtract tire speed due to rotation
wheel_vel_x-=(wheel_angvel*params.rad)
# find magnitude of combined slip
v_slip=(wheel_vel_x.^2+wheel_vel_y.^2).^0.5.+eps(1.)
# add small delta to prevent Inf result or huge slip ratio at low speeds - fix me...
slip=v_slip./(wheel_vel_X .+0.1)
# compute ratio of lateral to longitudinal, 1=lateral, 0=long
ratio=(wheel_vel_y./v_slip).^2

#println(slip)
#println(ratio)

# compute normal forces, start with weight
nrm_frc=[params.Zf,params.Zr,params.Zf,params.Zr]
# add deflection from bounce, roll, pitch times the suspension stiffness
defln=pos[3].+roll*params.t/2*[1,1,-1,-1]+pitch*[-params.a,params.b,-params.a,params.b]
nrm_frc-=[params.kf,params.kr,params.kf,params.kr].*defln
# add the damping forces
defln_rate=[glbl_vel[3],glbl_vel[3],glbl_vel[3],glbl_vel[3]]+ang_vel[1]*params.t/2*[1,1,-1,-1]+ang_vel[2]*[-params.a,params.b,-params.a,params.b]
nrm_frc-=[params.cf,params.cr,params.cf,params.cr].*defln_rate
# add the anti-roll bar terms
nrm_frc-=roll*[params.krf,params.krr,-params.krf,-params.krr]*params.t^2/2

# call tire model, define force as zero to start
lat_frc=[0.,0.,0.,0.]
long_frc=[0.,0.,0.,0.]

# total vertical force at tire
vert_frc=nrm_frc+[-1,-1,1,1].*jk_frc
off=vert_frc .<0
# can't be negative
nrm_frc[off] .= -([-1,-1,1,1].*jk_frc)[off]
nrm_frc .+=1.

tire_frc=tire_comb(slip,nrm_frc+[-1,-1,1,1].*jk_frc,ratio,camber,params) # Tire model
tire_frc_pus=tire_frc./v_slip

# tire force in tire x
long_frc_x=-wheel_vel_x.*tire_frc_pus
# tire force in tire y
lat_frc_y=-wheel_vel_y.*tire_frc_pus

# transform back to vehicle coordinate system, rear no steer
long_frc[[2,4]]=long_frc_x[[2,4]]
lat_frc[[2,4]]=lat_frc_y[[2,4]]

# overwrite front wheel w rotation calc
# notice opposite sign of sin(), transforming back to vehicle frame
long_frc[[1,3]]=long_frc_x[[1,3]]*cdelta-lat_frc_y[[1,3]]*sdelta
lat_frc[[1,3]]=lat_frc_y[[1,3]]*cdelta+long_frc_x[[1,3]]*sdelta

# jacking force, add using a low pass filter, otherwise numerically stiff, integrator very slow
jk_frc_dot=20*(lat_frc.*[params.hrf,params.hrr,params.hrf,params.hrr]/(params.t/2)-jk_frc)

# add rolling resistance
froll=nrm_frc*(0.0136+5.2*10^-7*vel[1]^2)
# set to zero if rolling backwards, to prevent issues on start
if vel[1]<0
	faero=0
	froll=[0,0,0,0]
end
long_frc-=froll

# compute aero drag, add in later
faero=0.5*1.23*params.cod*params.farea*vel[1]^2

# longitudinal speed control, tanh function smooths out on/off behavour of throttle, gentle slope around zero, much steeper away from zero, smoothly limited to +/- 1
throttle_dot=20*(tanh(0.2*(u_ref-vel[1])^3)-throttle)
brake=-throttle
# limit min throttle, brake
throttle < 0 && (throttle=0)
brake < 0 && (brake=0)

# find correct gear
gear=findnext(vel[1].<params.vmax,1)
# if going above redline in top gear, use top gear (it happens sometimes, not sure quite how, slip ratio?)
if isnothing(gear)
	gear=length(params.e)
end

# average wheel speed
wf=(wheel_angvel[1]+wheel_angvel[3])/2
wr=(wheel_angvel[2]+wheel_angvel[4])/2
# convert to rpm, find engine speed
# if FWD, use front wheel speed, if RWD, use rear wheel speed
if params.wtf==1
	we=wr*params.ex*params.e[gear]*30/pi
elseif params.wtf==-1
	we=wf*params.ex*params.e[gear]*30/pi
else
	we=(wf+wr)*params.ex*params.e[gear]*15/pi
end

# if engine speed below min speed, use min speed, otherwise Interpolations error
if we<params.revs[1]
	we=params.revs[1]
end

# add clutch torque, for low speed starts, launch control
ct=0
if gear==1 && we<params.launchrpm
	ct=(params.launchrpm-we)*0.05
	we=params.launchrpm
end
# limit engine speed, close throttle at 500 rpm above redline
if we>params.redline
	throttle=throttle-(we-params.redline)/500.
	if throttle<0
		throttle=0
	end
	we=params.redline
end

# fit torque curve to data
te=CubicSplineInterpolation(params.revs,params.torque)
# engine torque from curve fit, plus clutch torque
tq=ct+throttle*te(we)

# convert engine torque to axle torque
if params.wtf==1
	ax_tq=tq*params.eff*params.ex*params.e[gear]*[0.,0.5,0.,0.5]
elseif params.wtf==-1
	ax_tq=tq*params.eff*params.ex*params.e[gear]*[0.5,0.,0.5,0.0]
else
	ax_tq=tq*params.eff*params.ex*params.e[gear]*[0.25,0.25,0.25,0.25]
end

# compute brake torques
brake_m=1.5*params.mass*params.g*params.rad*brake
# divide brake torque front/rear by weighting
brake_tq=0.5*[params.fbf,(1-params.fbf),params.fbf,(1-params.fbf)].*tanh.(3*wheel_angvel)*brake_m

# find rotation matrix to vehicle frame, only roll and pitch needed
inv_rot_mtx=([c_pitch 0 -s_pitch;
	s_roll*s_pitch c_roll s_roll*c_pitch;
	c_roll*s_pitch -s_roll c_roll*c_pitch])

# compute acceleration from forces, local rotating frame!
acc=inv_rot_mtx*[sum(long_frc)-faero,sum(lat_frc),sum(nrm_frc+[-1,-1,1,1].*jk_frc)-params.mass*params.g]/params.mass-skew(ang_vel)*vel

# compute ang acceleration from moments, local rotating frame!
roll_m=(nrm_frc[1]+nrm_frc[2]-nrm_frc[3]-nrm_frc[4])*params.t/2+sum(lat_frc)*(params.hg-(params.hrf*params.b+params.hrr*params.a)/(params.a+params.b))
pitch_m=-(nrm_frc[1]+nrm_frc[3])*params.a+(nrm_frc[2]+nrm_frc[4])*params.b-sum(long_frc)*params.hg
yaw_m=(lat_frc[1]+lat_frc[3])*params.a-(lat_frc[2]+lat_frc[4])*params.b+(-long_frc[1]-long_frc[2]+long_frc[3]+long_frc[4])*params.t/2

ang_acc=params.I\(inv_rot_mtx*[roll_m,pitch_m,yaw_m]-(skew(ang_vel)*params.I*ang_vel))

# compute wheel ang acceleration
wheel_angacc=(ax_tq-brake_tq-params.rad*long_frc_x)/params.Iw

# return slopes to ODE solver
# return extra data after ODE solved
if(~flag)
	zdot.=[glbl_vel;euler_rates;acc;ang_acc;wheel_angacc;jk_frc_dot;vel[1];delta_dot;throttle_dot]
else
	zdot.=[acc;ang_acc;u_ref;path_error;heading_error;delta;throttle;brake;we;gear;camber;long_frc;lat_frc;nrm_frc;slip] # extra outputs
end

end
