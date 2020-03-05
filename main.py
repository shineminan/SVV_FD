# the main file
w = 60500  # N, standard aicraft mass
mdotfs = 0.048  # kg/second standard engine fuel flow per engine
rho0 = 1.225 # kg/m3 standard air density

lamda = -0.0065         # temperature gradient in ISA [K/m]
Temp0  = 288.15          # temperature at sea level in ISA [K]
R      = 287.05          # specific gas constant [m^2/sec^2K]
g      = 9.81            # [m/sec^2] (gravity constant)

S = 30  # area
cbar = 2.0569  # MAC
b = 15.911  # span

cd0 = 0.04
clalpha = 5.084
e = 0.8
kxx2 = 0.019
kyy2 = 1.3925
kzz2 = 0.042
kxz = 0.002

#  Linear model stability derivatives – symmetric flight
cxu = -0.0279
cxalpha = -0.4797
cxalphadot = 0.0833
cxq = -0.2817
cxdelta = -0.0373

czu = -0.3762
czalpha = -5.7434
czalphador = -0.0035
czq = -5.6629
czdelta = -0.6961

cm0 = 0.0297
cmu = 0.0699
cmalpha = -0.5626
cmalphadot = 0.1780
cmq = -8.7941
cmdelta = -1.1642
cmtc = -0.0064

miuc =