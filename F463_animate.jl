using DelimitedFiles
function F463_animate(lptime,nums,track;filename="history.html")

##
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by the
## Free Software Foundation; either version 2, or (at your option) any
## later version.
##
## This is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##

## Based on E. Grossmann's vrml_cyl.m, modified to generate x3d format code, rather than VRML
## Modification by Jiangtao Yu, Bruce Minaker, 2020
## Original Author: Etienne Grossmann <etienne@cs.uky.edu>

println("Writing animation...")

#locate first three ranks in the results
v_win=partialsortperm(lptime,1:3) # sort out the three lesat laptime and locate the index
nums_win=nums[v_win] # find the correspond student_ID for three winners
laptime=lptime[v_win] # same as above for laptimes

#Define parameters
x3d_chasis=["" "" ""]; # predefine the string used to draw car bodies
tck="" # track path data
tck_idx="" #index for tracking path points
s_tck="" #pre-define the track path string
n=size(track)[1]
scal=""
tme=""
s_total=""
s_mtx=["" "" ""]

#Draw
#ground (using the x3d_grnd function )
x3d_grnd=x3d_pnt([0,0,0],cubes=true,rad=[50,50,0.01],col=[0.25,0.25,0.25],tran=0.5);

#vehicle
x3d_chasis[1]=x3d_pnt([0,0,0],cubes=true,rad=[2.5,1.5,1.2],col=[1,0,0],tran=0.3);
x3d_chasis[2]=x3d_pnt([0,0,0],cubes=true,rad=[2.5,1.5,1.2],col=[0,1,0],tran=0.3);
x3d_chasis[3]=x3d_pnt([0,0,0],cubes=true,rad=[2.5,1.5,1.2],col=[0,0,1],tran=0.3);

# Build string for each winner
for i=1:3
	pstn="" # position history
	rntn="" # Rotation history

# load data
grp=nums_win[i] # student bumbers for each winner
folder="$(grp[1])_$(grp[2])" # locate the corresponde folder according to winners' ID
cd("results/$(folder)")
time_hstr=readdlm("time_history.txt"); # read the data
tout=time_hstr[:,7]; # real time
lcn=time_hstr[:,1:3]'; # location
rtn=time_hstr[:,4:6]'; # rotation
time_interval=tout[end]; # laptime
time=tout;
time/=time_interval; # devide the real time by the laptime to switch to key format that required in x3dom
m=length(time) # number of points in the x3dom

# writting the string for location history
for i=1:m-1
	pstn*="$(lcn[1,i]) $(lcn[2,i]) $(lcn[3,i]),\n" # string for each key point (moment)
end
pstn*="$(lcn[1,m]) $(lcn[2,m]) $(lcn[3,m])\n" #last point

# Writting the string for rotation history
rntn*="0 0 0 0,\n" # starting from "0 0 0 0"
for i=2:m
	# Roll Pitch Yaw
	roll=rtn[1,i];
	pitch=rtn[2,i];
	yaw=rtn[3,i];
	s_roll=sin(roll);
	c_roll=cos(roll);
	s_pitch=sin(pitch);
	c_pitch=cos(pitch);
	s_yaw=sin(yaw);
	c_yaw=cos(yaw);

	# Rotation Matrix that required to find euler's parameters
	S_ao=[c_pitch*c_yaw c_roll*s_yaw+s_roll*s_yaw*s_pitch s_yaw*s_roll-c_roll*s_pitch*c_yaw;
		-c_pitch*s_yaw c_roll*c_yaw-s_roll*s_yaw*s_pitch s_roll*c_yaw+c_roll*s_pitch*s_yaw;
		s_pitch -s_roll*c_pitch c_roll*c_pitch]
		trS=S_ao[1,1]+S_ao[2,2]+S_ao[3,3];

		# Euler's parameters
		eu4=sqrt((trS+1)/4);
		eu1=(S_ao[3,2]-S_ao[2,3])/(4*eu4);
		eu2=(S_ao[1,3]-S_ao[3,1])/(4*eu4);
		eu3=(S_ao[2,1]-S_ao[1,2])/(4*eu4);

		# Absolute 3D rotation angle and its axis unit vector, this is the format to represent a 3D rotation in x3dom
		theta=2*(acos(eu4)); # absolute angle
		lam1=eu1/(sin(theta/2)); # unit vector of rotation axis
		if abs(lam1)==0 # delete the negative values
			lam1=0
		end
		lam2=eu2/(sin(theta/2)); # unit vector of rotation axis
		if abs(lam2)==0
			lam2=0
		end
		lam3=-eu3/(sin(theta/2)); # unit vector of rotation axis
		if abs(lam3)==0
			lam3=0
		end

		rntn*="$lam1 $lam2 $lam3 $theta,\n" # final format for presenting a rotating motion in x3dom
end

# Define Key
for i=1:m-1
	tme*="$(time[i])," # "Key"
end
	tme*="$(time[m])"

# Writting the chassis strings in the format of x3dom interpolation
s="<PositionInterpolator DEF='chasis_psn_$(i)' keyValue='\n"*pstn*"' key='"*tme*"'></PositionInterpolator>\n"
s*="<OrientationInterpolator DEF='chasis_rtn_$(i)' keyValue='\n"*rntn*"' key='"*tme*"'></OrientationInterpolator>\n"

s*="<Transform DEF='chasis_$(i)' >\n"
s*=x3d_chasis[i]
s*="</Transform>\n"

s*="<ROUTE fromNode='IDtimer' fromField='fraction_changed' toNode='chasis_psn_$(i)' toField='set_fraction'></ROUTE>\n"
s*="<ROUTE fromNode='IDtimer' fromField='fraction_changed' toNode='chasis_rtn_$(i)' toField='set_fraction'></ROUTE>\n"
s*="<ROUTE fromNode='chasis_psn_$(i)' fromField='value_changed' toNode='chasis_$(i)' toField='set_translation'></ROUTE>\n"
s*="<ROUTE fromNode='chasis_rtn_$(i)' fromField='value_changed' toNode='chasis_$(i)' toField='set_rotation'></ROUTE>\n"

s
s_total*=s;
s_mtx[i]=s; # Save the sttring to the string matrix
cd("../..")
end

# Writting the ground strings in the format of x3dom interpolation
s_total*=x3d_grnd;
s_mtx[3]*=x3d_grnd;

# Writting the track map strings in the format of x3dom interpolation
for i=1:n
	tck*="$(track[i,1]) $(track[i,3]) $(-track[i,2]),\n"
end
	idx=hcat(0:1:n-1)'
for i=1:n
	tck_idx*="$(idx[i]), "
end
s_tck*="<IndexedLineSet coordIndex='\n"*tck_idx*"'>\n"
s_tck*="<Coordinate point='\n"*tck*"'>\n"
s_tck*="</IndexedLineSet>\n"
s_tck

# make the folde to save the animation
dir = joinpath("results", "competition_x3d")
~isdir(dir) && (mkdir(dir))

# call x3d_save function to finalize the file generation
x3d_save(s_mtx,s_tck,joinpath(dir,filename),laptime)

println("Animation written.")

end
