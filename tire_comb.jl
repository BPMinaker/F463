function tire_comb(slip,load,ratio,camber,params)

slip_m=slip.*(180/pi*ratio+100*(1. .-ratio))

C=params.a_mtm[1]*ratio+params.b_mtm[1]*(1. .-ratio)
mu_p=(params.a_mtm[2]*ratio+params.b_mtm[2]*(1. .-ratio)).*load/1000+ (params.a_mtm[3]*ratio+params.b_mtm[3]*(1. .-ratio))
D=mu_p.*load/1000

Ea=params.a_mtm[7]*load/1000. .+params.a_mtm[8]
Eb=params.b_mtm[7]*(load/1000.).^2. .+params.b_mtm[8]*load/1000. .+params.b_mtm[9]
E=ratio.*Ea+(1. .-ratio).*Eb

Ba=params.a_mtm[4]*sin.(2*atan.(load/1000. /params.a_mtm[5])).*(1. .-params.a_mtm[6]*abs.(camber))./C./D;
Bb=(params.b_mtm[4]*(load/1000).^2+params.b_mtm[5]*load/1000).*exp.(-params.b_mtm[6]*load/1000)./C./D;
B=ratio.*Ba+(1. .-ratio).*Bb;

Sh=0
trac=D.*sin.(C.*atan.(B.*(1. .-E).*(slip_m .+Sh) .+E.*atan.(B.*(slip_m .+Sh))))

# trac[load .< 0] .=0.

trac

end  ## Leave

#Sh=params.b_mtm(10)*load/1000+params.b_mtm(11);


#
# slip=-1:0.01:1 # converted to % in tire.jl
#
# # longitudinal
# Fx2000=tire_comb(collect(slip),2000*ones(length(slip)),0*ones(length(slip)),0*ones(length(slip)),params)
# Fx4000=tire_comb(collect(slip),4000*ones(length(slip)),0*ones(length(slip)),0*ones(length(slip)),params)
# Fx6000=tire_comb(collect(slip),6000*ones(length(slip)),0*ones(length(slip)),0*ones(length(slip)),params)
#
# display(plot(slip,[Fx2000 Fx4000 Fx6000],xlabel="Slip",ylabel="Longitudinal force [N]",label=["2000 N" "4000 N" "6000 N"]))
#
#
# slip=-12:0.006:12
# # lateral
# Fy2000=tire_comb(pi/180*collect(slip),2000*ones(length(slip)),1*ones(length(slip)),0*ones(length(slip)),params)
# Fy4000=tire_comb(pi/180*collect(slip),4000*ones(length(slip)),1*ones(length(slip)),0*ones(length(slip)),params)
# Fy6000=tire_comb(pi/180*collect(slip),6000*ones(length(slip)),1*ones(length(slip)),0*ones(length(slip)),params)
# #
# display(plot(slip,[Fy2000 Fy4000 Fy6000],xlabel="Sideslip [degrees]",ylabel="Lateral force [N]",label=["2000 N" "4000 N" "6000 N"]))
