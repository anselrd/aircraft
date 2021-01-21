function [value,isterminal,direction] = golf_ball_ground_event(~,y)
%Event function for integration of Lyapunov-type orbits. Sets the stopping
%condition to the crossing of the x axis from above.

% Set the event function value
value = y(3);

% The condition is terminal
isterminal = 1;

% Only look for zeros when z is increasing
direction = 1;

end