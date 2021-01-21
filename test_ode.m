function dy = test_ode(x,y)

u = y(1);
v = y(2);
w = y(3);

dy(1) = 3*w + z;
dy(2) = x^2;
dy(3) = -u + v;

dy = dy';

end