function R = Rz(theta)
%RZ
%    R = RZ(THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    21-Aug-2019 16:41:31

t2 = cos(theta);
t3 = sin(theta);
R = reshape([t2,t3,0.0,-t3,t2,0.0,0.0,0.0,1.0],[3,3]);
