function plot_longitudinal_states(t,y)

figure('pos',[560 900 650 840])
subplot(811)
plot(t,y(:,4))
ylabel('u [m/s]')
title('State Evolution')
grid on
subplot(812)
plot(t,y(:,5))
ylabel('v [m/s]')
grid on
subplot(813)
plot(t,y(:,6))
ylabel('w [m/s]')
grid on
subplot(814)
plot(t,y(:,10))
ylabel('p [rad/s]')
grid on
subplot(815)
plot(t,y(:,11))
ylabel('q [rad/s]')
grid on
subplot(816)
plot(t,y(:,12))
ylabel('r [rad/s]')
grid on
subplot(817)
plot(t,y(:,7))
ylabel('\phi [deg]')
grid on
subplot(818)
plot(t,y(:,8))
ylabel('\theta [deg]')
xlabel('Time [s]')
grid on