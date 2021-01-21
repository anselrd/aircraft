function plot_longitudinal_states_linear(t,y,trim)

figure('pos',[560 900 650 840])
subplot(711)
plot(t,y(:,4)+trim.u)
ylabel('u [m/s]')
title('Linear State Evolution')
grid on
subplot(712)
plot(t,y(:,5)+trim.v)
ylabel('v [m/s]')
grid on
subplot(713)
plot(t,y(:,6)+trim.w)
ylabel('w [m/s]')
grid on
subplot(714)
plot(t,y(:,11)+trim.q)
ylabel('q [rad/s]')
grid on
subplot(715)
plot(t,y(:,8)+trim.theta)
ylabel('\theta [deg]')
xlabel('Time [s]')
grid on
subplot(716)
plot(t,y(:,1)+t*trim.u)
ylabel('x^E [m]')
xlabel('Time [s]')
grid on
subplot(717)
plot(t,y(:,3))
ylabel('z^E [m]')
xlabel('Time [s]')
grid on