# Edit this file to enter your team info and vehicle specs
# Do not alter the format of this file!!!

student_names=["test_group4_1" "test_group4_2"]
student_numbers=["000000007" "000000008"]
group_name=["Laws of Motion"]

params.engine_type=1 # Enter your engine choice here
# 1 means engine 1 (higher redline)
# 2 means engine 2 (more torque)

params.e=[4.5,3.6,2.9,2.2,1.7] # Enter your 5 or 6 gear ratios here, starting from _first_ gear, i.e., in the default values, 1st gear is 5.0:1, note that the defaults are a poor choice
params.ex=4.4 # Enter your axle final drive ratio here

# centre of mass location
params.fwf=0.46# front weight fraction, from 0.05 to 0.95
# 0 means all weight on rear axle
# 1 means all weight on front axle
# 0.5 means equal weight on both axles

# suspension stiffness [N/m]
# force per deflection, each corner
params.kf=78000
params.kr=150000

# suspension damping [Ns/m]
# force per deflection rate, each corner
params.cf=11000
params.cr=13000

# roll bar stiffness [N/m]
# force per deflection difference across axle
params.krf=80000
params.krr=30000

# roll centre height
params.hrf=0.0001
params.hrr=0.0001

# camber sensitivity coefficient from 0 to 1.
# 1 means that the tire camber will equal the roll, independent of bounce
# 0 means that the camber is independent of roll, equals bounce of axle divided by half track width
# between 0 and 1 weighted average
params.df=0.9
params.dr=0.9
# braking system parameters
params.fbf=0.7 # front brake fraction, from 0 to 1
# 0 means rear brakes only
# 1 means front brakes only
# 0.5 equal brake torques on both axles

# driver model
params.acc_lat_max=0.4 # [g's] maximum lateral acceleration, used to compute target speeds based on track curvature
params.acc_brake_max=0.3 # [g's] maximum braking acceleration, used to decide when to apply the brakes during corner entry
params.acc_drive_max=1.2# [g's] maximum driving acceleration, used to decide how much throttle on corner exit

params.maxv=100 # Driver model max allowed speed (m/s)
