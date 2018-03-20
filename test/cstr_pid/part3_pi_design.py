# from identification
Kp = 2.16288502017
taup = 0.913444964569
thetap = 0.000121628824381

# design PI controller
tauc = max(0.1*taup,0.8*thetap)
Kc = (1.0/Kp)*(taup/(thetap+tauc))
tauI = taup

print('Kc: ' + str(Kc))
print('tauI: ' + str(tauI))
