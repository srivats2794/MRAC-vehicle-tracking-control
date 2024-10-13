function [RMS_y,E_y]=calc_LateralDeviation(X,Y,X_ref,Y_ref,psi,simTs,ctrlTs)

indices= 1:round(ctrlTs/simTs):length(X);

indices(2:end)=indices(2:end)-1;

for i=1:length(indices)
    X_diff= X(indices(i))-X_ref(i);
    Y_diff= Y(indices(i))-Y_ref(i);
   
    Ey(i)= Y_diff*cos(psi(indices(i)))-X_diff*sin(psi(indices(i)));
end
RMS_y=sqrt(sum(Ey.^2)/length(Ey));

disp(['Average Lateral Deviation = ' num2str(RMS_y)]);
end