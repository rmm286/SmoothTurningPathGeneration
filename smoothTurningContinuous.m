function output = smoothTurningContinuous(input)

t = input.phase.time;

x = input.phase.state(:,1);
y = input.phase.state(:,2);
th = input.phase.state(:,3);
k = input.phase.state(:,4);
v = input.phase.state(:,5);

kDot = input.phase.control(:,1);
vDot = input.phase.control(:,2);

dx = v.*cos(th);
dy = v.*sin(th);
dth = v.*k;
dk = kDot;
dv = vDot;

output.dynamics = [dx, dy, dth, dk, dv];

end
