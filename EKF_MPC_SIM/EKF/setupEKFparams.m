function obj= setupEKFparams(vehicle,controlTs)
    MeasUC= [0.1*0.0174533;1]; % YawRate, Y-Acceleration
    ProcUC= [0.01*0.0174533;100;100]; % delta,C_f,C_r
    obj=PARAMETER_ESTIMATOR(vehicle,controlTs,MeasUC,ProcUC,0.3,0.3);
end