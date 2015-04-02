% script to check roll-pitch yaw convention and compute berm centroid
% measurement model

% notation:
% phi = roll
% theta = pitch
% psi = yaw

% define syms for sines and cosines of roll, pitch, and yaw
syms phi theta psi

% define single axis rotations
Rx = [1  0   0
      0 cos(phi) -sin(phi)
      0 sin(phi) cos(phi)];
Ry = [cos(theta)  0 sin(theta)
       0  1  0
     -sin(theta)  0 cos(theta)];
Rz = [cos(psi) -sin(psi)  0
      sin(psi)  cos(psi)  0
       0   0  1];
   
% assume RPY means roll then pitch then yaw relative to fixed frame
%  ==> get overall rotation by multiplying successively on left
RL_V = Rz * Ry * Rx;

% define syms for landmark centroid x and vehicle position d, all of these
% are expressed in the local frame
syms xx xy xz dx dy dz;
x = [xx; xy; xz];
d = [dx; dy; dz];

% now expand the measurement model
RV_L = transpose(RL_V);
h = RV_L*(x-d)

% compute measurement Jacobian (brute force method)
p = [dx, dy, dz, phi, theta, psi];
H1 = transpose(gradient(h(1),p));
H2 = transpose(gradient(h(2),p));
H3 = transpose(gradient(h(3),p));
Hbf = [H1; H2; H3];
% now display J element by element
for row = 1:3
    for col = 1:6
        disp(['Hbf(',num2str(row),',',num2str(col),') = ']);
        disp(Hbf(row,col));
    end
end

% compute measurement Jacobian (matrix math method)
% first compute partials of R
R = RV_L;
dRdphi = diff(R,phi)
dRdtheta = diff(R,theta)
dRdpsi = diff(R,psi)

% construct measurement Jacobian
Hmm = [-R, dRdphi*(x-d), dRdtheta*(x-d), dRdpsi*(x-d)];

% double check that brute force matches the matrix math
shouldBeZeros = Hbf - Hmm

        
        


   
