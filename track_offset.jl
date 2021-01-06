using Polynomials,LinearAlgebra

function track_offset(pos,theta,distance,vx,params)

# load track data
#tracklen=size(params.course,1);  ## Find number of rows
idx=Int(round(distance/5)+1)  ## Use distance to find approximate location (5 is ds)
rot_mtx=[cos(theta) sin(theta); -sin(theta) cos(theta)]  ## Build rotation matrix
d_last=norm(params.course[idx,1:2]-pos)  ## Find distance to last point

# Find closest point by looking ahead
d_next=d_last  ## Choose to make loop eval at least once
nidx=idx  ## Start looking at current idx
while d_next<=d_last ##&& nidx<tracklen-2)  ## If the next point is closer than the last, keep looking forward
	d_last=d_next
	nidx+=1
	d_next=norm(params.course[nidx,1:2]-pos)  ## Compute distance from track point to current point
end

del=nidx-1-idx  ## Number of points we move forward
idx=nidx-1  ## If the last nidx broke the loop, it was one step too far forward, but reset idx

if idx>1 && del==0  ## Now check behind also, if we didn't move forward
	d_next=d_last  ## Choose to make loop eval once
	nidx=idx
	while d_next<=d_last ## && nidx>1)
		d_last=d_next
		nidx=nidx-1
		d_next=norm(params.course[nidx,1:2]-pos)  ## If the next point is closer than the last, keep looking forward
	end
	idx=nidx+1  ## If the last nidx broke the loop, it was one step too far back, but reset idx
end

ptb=idx-1  ## Make sure point behind is not before start?
indc=[ptb;idx]

d_ahead=5.0+0.5*vx  ## Look ahead distance

ni=ceil(d_ahead/5.0)  ## Number of points (how to know 5?, fix me...)

pta=idx+1

for i=1:ni  ## Add points to index
	indc=[indc;pta]
	pta=pta+1
end
if indc[1]==0
	indc.+=1
end

pts=[0,0.1,0.2,0.3,0.4,0.6,0.8,1]*d_ahead

temp=params.course[indc,1:2]
temp=(rot_mtx*(temp' .-pos))'  ## Convert to vehicle coordinate system

p=fit(temp[:,1],temp[:,2],2)  ## Fit a curve through them
offset=p.(pts)  ## Calculate offset

temp[:,2]=params.course[indc,5]  ## Find desired angle
p=fit(temp[:,1],temp[:,2],2)  ## Fit a parabola through them
dpsi=p.(pts) .- theta

temp[:,2]=params.course[indc,6]  ## Find desired speeds
p=fit(temp[:,1],temp[:,2],2)  ## Fit a parabola through them
u_ref=p(0)

temp[:,2]=params.course[indc,4]  ## Find desired curvature
p=fit(temp[:,1],temp[:,2],2)  ## Fit a parabola through them
curv=p(0)

offset,dpsi,u_ref,curv

end  ## Leave
