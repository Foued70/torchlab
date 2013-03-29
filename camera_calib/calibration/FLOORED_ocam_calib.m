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

% Run this and follow prompts.

FLOORED_data_calib;
click_calib;

width_target=1024;

scale=width/width_target;

% find scaled down versions

nx_scaled=nx/scale;
ny_scaled=ny/scale;
width_scaled=width/scale;
height_scaled=height/scale;
xc_scaled=xc/scale;
yc_scaled=yc/scale;
Xt_scaled=Xt;
Yt_scaled=Yt;
Xp_abs_scaled=Xp_abs./scale;
Yp_abs_scaled=Yp_abs./scale;

% set original so we can get back

nx_orig=nx;
ny_orig=ny;
width_orig=width;
height_orig=height;
xc_orig=xc;
yc_orig=yc;
Xt_orig=Xt;
Yt_orig=Yt;
Xp_abs_orig=Xp_abs;
Yp_abs_orig=Yp_abs;

% set current to scaled

nx=nx_scaled;
ny=ny_scaled;
width=width_scaled;
height=height_scaled;
xc=xc_scaled;
yc=yc_scaled;
Xt=Xt_scaled;
Yt=Yt_scaled;
Xp_abs=Xp_abs_scaled;
Yp_abs=Yp_abs_scaled;

% run calibration on scaled values

calibration;
findcenter;
calibration;

% set scaled params

RRfin_scaled=RRfin;
ocam_model_scaled=ocam_model;
ss_scaled=ss;
xc_scaled=xc;
yc_scaled=yc;

% set up initial orig params

RRfin_orig=RRfin_scaled;
xc_orig=xc_scaled*scale;
yc_orig=yc_scaled*scale;
ss_orig=ss_scaled.*[scale, 1, 1/scale, 1/(scale^2), 1/(scale^3)]';
ocam_model_orig=ocam_model_scaled;
ocam_model_orig.xc=ocam_model_scaled.xc*scale;
ocam_model_orig.yc=ocam_model_scaled.yc*scale;
ocam_model_orig.ss=ocam_model_scaled.ss.*[scale, 1, 1/scale, 1/(scale^2), 1/(scale^3)]';

% go back to original

nx=nx_orig;
ny=ny_orig;
width=width_orig;
height=height_orig;
xc=xc_orig;
yc=yc_orig;
Xt=Xt_orig;
Yt=Yt_orig;
Xp_abs=Xp_abs_orig;
Yp_abs=Yp_abs_orig;
ocam_model=ocam_model_orig;
ss=ss_orig;
RRfin=RRfin_orig;

FLOORED_findcenter_norrfin;
FLOORED_optimizefunction;

% fit relevant polynomials

pol=findinvpoly(ocam_model.ss, sqrt((width/2)^2 + (height/2)^2));

ocam_model.pol=pol;

[pol, invpol, errsq, maxerr, inverrsq, invmaxerr, rho, theta, XRealCord, YRealCord, ZRealCord, XImg, YImg]=FLOORED_findrho2theta(ocam_model, RRfin, Xt, Yt, Xp_abs, Yp_abs, ima_proc);

ocam_model.r2t=pol;
ocam_model.t2r=invpol;

% set original

xc_orig=xc;
yc_orig=yc;
ss_orig=ss;
ocam_model_orig=ocam_model;
RRfin_orig=RRfin;

reprojectpoints;
ocam_model_orig

% after calibration, undistort using FLOORED_undistort_image
% for example, to undistort DSC_0044.jpg with output image size of 2000h x 1000w,
% type FLOORED_undistort_image('DSC_0044', ocam_model_orig, 2000, 1000);

% save out the model
save "ocam_model.txt" ocam_model_orig

