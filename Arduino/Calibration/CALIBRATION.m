%% mapping of motor input (data_v) to the velocity in cm/s
subplot(2,2,[1 2])
f_vel=fit(Velocitycalibration(:,6),Velocitycalibration(:,4),'poly1')
plot(Velocitycalibration(:,6),Velocitycalibration(:,4),'.',MarkerSize=5)
hold on
plot(f_vel)
ylabel('data v')
xlabel('velocity(cm/s)')

%% mapping of motor input (data_rz) to the angular rate
subplot(2,2,3)
f_rot_ccw=fit(Angular_rate_Calibration(13:24,5),Angular_rate_Calibration(13:24,3),'poly1')
plot(Angular_rate_Calibration(13:24,5),Angular_rate_Calibration(13:24,3),'.',MarkerSize=5)
hold on 
plot(f_rot_ccw)
ylabel('data rz ccw')
xlabel('angular rate(deg/s)')


subplot(2,2,4)
f_rot_cw=fit(Angular_rate_Calibration(1:12,5),Angular_rate_Calibration(1:12,3),'poly1')
plot(Angular_rate_Calibration(1:12,5),Angular_rate_Calibration(1:12,3),'.',MarkerSize=5)
hold on 
plot(f_rot_cw)
ylabel('data rz cw')
xlabel('angular rate(deg/s)')
