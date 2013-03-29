%WORLD2CAM projects a 3D point on to the image
%   m=WORLD2CAM(M, ocam_model) projects a 3D point on to the
%   image and returns the pixel coordinates.
%   
%   M is a 3xN matrix containing the coordinates of the 3D points: M=[X;Y;Z]
%   "ocam_model" contains the model of the calibrated camera.
%   m=[rows;cols] is a 2xN matrix containing the returned rows and columns of the points after being
%   reproject onto the image.
%   
%   Copyright (C) 2006 DAVIDE SCARAMUZZA   
%   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

function m=FLOORED_world2cam_thoby(M, ocam_model)
ss = ocam_model.ss;
xc = ocam_model.xc;
yc = ocam_model.yc;
width = ocam_model.width;
height = ocam_model.height;
c = ocam_model.c;
d = ocam_model.d;
e = ocam_model.e;

%[x,y]  = omni3d2pixel(ss,M,width, height);
theta=atan(-sqrt(M(1,:).^2 + M(2,:).^2)./M(3,:));
%rho=polyval(ocam_model.t2r, theta);
fpix=width*(10.58/23.6);
k1=1.47;
k2=0.713;
rho=k1*fpix*sin(k2*theta);
x=rho.*M(1,:)./sqrt(M(1,:).^2 + M(2,:).^2);
y=rho.*M(2,:)./sqrt(M(1,:).^2 + M(2,:).^2);

m(1,:) = x*c + y*d + xc;
m(2,:) = x*e + y   + yc;