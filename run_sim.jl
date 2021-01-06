
# load library to write to text file
using DelimitedFiles

# note all these includes don't actually run anything, just define functions that will be called later on
include("solver.jl") # high level solver
include("prelim.jl") # compute shift points
include("eqn_of_motion.jl") # equations of motion, driver model
include("tire_comb.jl") # tire model
include("track.jl")  # define the track
include("track_offset.jl") # track location
include("plot_results.jl") # plot results
include("x3d_pnt.jl"); # define the car body in animation
include("x3d_save.jl"); # writing the racing time history into x3dom file (.html)
include("F463_animate.jl") # the function to save the animation of time history

include("vehicle_specs.jl") # this will define the custom variable type called 'props', and initialize one instance called 'params' that holds all the default values of the vehicle, driver, and track parameters

list = readdir("students") # read in the list of input files in the student folder
laptime = [] # empty variable to store laptime
names = [] # student names
nums = [] # student numbers

# for each team
for file in list
    println("Running ", file)
    # read student chosen params
    include(joinpath("students", file))

    # set engine data, compute shift speeds, p is plot of performance map
    p = prelim(params)
    # load track, and compute target velocity around it for driver
    track(params)

    # call solver to run
    sol, yout = solver(params)

    # print solver stats
    println(sol.destats)

    # record laptime, name, numbers
    push!(laptime, sol.t[end])
    println("Laptime is $(laptime[end]) seconds!")
    push!(names, student_names)
    push!(nums, student_numbers)

    # plot results in timestamped folder, name saved in "out"
    out = plot_results(sol, yout, p, student_numbers, params, disp = false)
    # save the input file there too
    cp(joinpath("students", file), joinpath(out, "my_specs.jl"))

end # loop over each team

# file name and track path for animation
track_x3d = [params.course[:, 1:2] zeros(size(params.course)[1])]
# write animation of top three fastest vehicles
F463_animate(laptime, nums, track_x3d, filename = "history.html")

writedlm("results.out", [laptime names nums])    # write the results

println("Done.")

