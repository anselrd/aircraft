function dx = LTI_aircraft_sim(~,x,A)

% compute the derivatives
dx = A*x(:);

end