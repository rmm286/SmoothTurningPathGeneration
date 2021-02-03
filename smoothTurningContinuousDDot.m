function output = smoothTurningContinuousDDot(input)

t = input.phase.time;

x = input.phase.state(:,1);
y = input.phase.state(:,2);
th = input.phase.state(:,3);
k = input.phase.state(:,4);
v = input.phase.state(:,5);
kDot = input.phase.state(:,6);
vDot = input.phase.state(:,7);

kDDot = input.phase.control(:,1);
vDDot = input.phase.control(:,2);

dx = v.*cos(th);
dy = v.*sin(th);
dth = v.*k; 
dk = kDot;
dv = vDot;
dkDot = kDDot;
dvDot = vDDot;

output.dynamics = [dx, dy, dth, dk, dv, dkDot, dvDot];

end