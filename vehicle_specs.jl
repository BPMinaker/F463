
Base.@kwdef mutable struct props
	mass::Float64 = 1600.
	I::Array{Float64,2} = [800 0 0;0 2000 0;0 0 2200]
	wb::Float64 = 2.6
	fwf::Float64 = 0.6

	a::Float64 = 0
	b::Float64 = 0

	t::Float64 = 1.5
	Iw::Float64 = 1.3

	hrf::Float64 = 0.1
	hrr::Float64 = 0.1
	hg::Float64 = 0.5

	df::Float64 = 0.5
	dr::Float64 = 0.5

	kf::Float64 = 10000
	kr::Float64 = 10000

	cf::Float64 = 2000
	cr::Float64 = 2000

	krf::Float64 = 5000
	krr::Float64 = 2000

	cod::Float64 = 0.35
	farea::Float64 = 3.2

	rad::Float64 = 17/2*0.0254+205*50/100000
	drive::String = "RearWheelDrive"
	wtf::Int64 = 1

	Zf::Float64 = 0
	Zr::Float64 = 0

	a_mtm::Vector{Float64} = [1.6929,-55.2084,1271.28,1601.8,6.4946,4.7966E-3,-0.3875,1,-4.5399E-2,4.2832E-3,8.6536E-2,-7.973,-0.2231,7.668,45.8764]
	b_mtm::Vector{Float64} = [1.65,-7.6118,1122.6,-7.36E-3,144.82,-7.6614E-2,-3.86E-3,8.5055E-2,7.5719E-2,2.3655E-2,2.3655E-2]

	g::Float64 = 9.81

	e::Vector{Float64} = [5.,4.,3.,2.,1.]
	ex::Float64 = 3.5 ## final drive ratio
	eff::Float64 = 0.9
	engine_type::Int64=1
	revs::StepRangeLen{Float64} = 1.:1.:1.
	torque::Vector{Float64} = [0.]
	redline::Float64 = 0.
	launchrpm::Float64 = 0.
	vmax::Vector{Float64} = [0.,0.,0.,0.,0.]

	fbf::Float64 = 0.65 # front brake fraction, from 0 to 1

	acc_lat_max::Float64 =0.3 # [g's] maximum lateral acceleration
	acc_brake_max::Float64 = 0.3 # [g's] maximum braking acceleration
	acc_drive_max::Float64 = 0.3 # [g's] maximum driving acceleration
	maxv::Float64 = 100. # Driver model max allowed speed (m/s)

	course::Array{Float64,2} = [0 0;0 0]

end

params=props()
