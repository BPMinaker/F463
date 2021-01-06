# Edit this file to enter your team info and vehicle specs
# Do not alter the format of this file!!!

student_names=["test_group1_1" "test_group1_2"]
student_numbers=["000000001" "000000002"]
group_name=["Laws of Motion"]

params.engine_type=2 # Enter your engine choice here
# 1 means engine 1 (higher redline)
# 2 means engine 2 (more torque)

params.e=[5,4,3,2,1] # Enter your 5 or 6 gear ratios here, starting from _first_ gear, i.e., in the default values, 1st gear is 5.0:1, note that the defaults are a poor choice
params.ex=3.5 # Enter your axle final drive ratio here

# centre of mass location
params.fwf=0.5 # front weight fraction, from 0.05 to 0.95
# 0 means all weight on rear axle
# 1 means all weight on front axle
# 0.5 means equal weight on both axles

# suspension stiffness [N/m]
# force per deflection, each corner
params.kf=15000
params.kr=16000

# suspension damping [Ns/m]
# force per deflection rate, each corner
params.cf=1000
params.cr=1000

# roll bar stiffness [N/m]
# force per deflection difference across axle
params.krf=2000
params.krr=1000

# roll centre height
params.hrf=0.1
params.hrr=0.1

# camber sensitivity coefficient from 0 to 1.
# 1 means that the tire camber will equal the roll, independent of bounce
# 0 means that the camber is independent of roll, equals bounce of axle divided by half track width
# between 0 and 1 weighted average
params.df=0.5
params.dr=0.5

# braking system parameters
params.fbf=0.7 # front brake fraction, from 0 to 1
# 0 means rear brakes only
# 1 means front brakes only
# 0.5 equal brake torques on both axles

# driver model
params.acc_lat_max=0.3 # [g's] maximum lateral acceleration, used to compute target speeds based on track curvature
params.acc_brake_max=0.3 # [g's] maximum braking acceleration, used to decide when to apply the brakes during corner entry
params.acc_drive_max=0.3 # [g's] maximum driving acceleration, used to decide how much throttle on corner exit

params.maxv=100 # Driver model max allowed speed (m/s)
