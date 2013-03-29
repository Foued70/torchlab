%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%   Copyright (C) 2006 DAVIDE SCARAMUZZA
%   
%   Author: Davide Scaramuzza - email: davsca@tiscali.it
%   
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; either version 2 of the License, or
%   (at your option) any later version.
%   
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%   
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
%   USA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function res=prova_leasqr(targInput, x, ss, width, height)

Xp_abs_centered=targInput(1,:)';
Yp_abs_centered=targInput(2,:)';
M=targInput(3:5,:)';

c=1;
d=0;
e=0;
num_points=size(M,1);
R=rodrigues([x(1),x(2),x(3)]);
T=[x(4),x(5),x(6)]';

Mc=R*M'+T*ones(1,num_points);
[xp,yp]=omni3d2pixel(ss,Mc, width, height);
%xp=xp1*c + yp1*d + xc;
%yp=xp1*e + yp1 + yc;

res=sqrt( (Xp_abs_centered-xp').^2+(Yp_abs_centered-yp').^2 );