%UNDISTORT unwrap part of the image onto a plane perpendicular to the
%camera axis
%   B = UNDISTORT(OCAM_MODEL, A, FC, DISPLAY)
%   A is the input image
%   FC is a factor proportional to the distance of the camera to the plane;
%   start with FC=5 and then tune the parameter to change the result.
%   DISPLAY visualizes the output image if set to 1; its default value is
%   0.
%   B is the final image
%   Note, this function uses nearest neighbour interpolation to unwrap the
%   image point. Better undistortion methods can be implemented using
%   bilinear or bicub interpolation.
%   Note, if you want to change the size of the final image, change Nwidth
%   and Nheight
%   Author: Davide Scaramuzza, 2009

function Nimg = FLOORED_undistort_image( fname, ocam_model, outheight, outwidth)

I1=imread(strcat(fname,'.jpg'));
%imwrite(I1, strcat(fname,'_in.jpg'));

if size(I1,1) > size(I1,2)
    if length(size(I1)) == 3
        IIn(:,:,1)=I1(:,:,1)';
        IIn(:,:,2)=I1(:,:,2)';
        IIn(:,:,3)=I1(:,:,3)';
    else
        IIn=I1';
    end;
    I1_out_tmp=undistort_image_help(ocam_model, IIn, outwidth, outheight);
    if length(size(I1)) == 3
        I1_out(:,:,1)=I1_out_tmp(:,:,1)';
        I1_out(:,:,2)=I1_out_tmp(:,:,2)';
        I1_out(:,:,3)=I1_out_tmp(:,:,3)';
    else
        I1_out=I1_out_tmp';
    end;
else
    IIn=I1;
    I1_out=undistort_image_help(ocam_model, IIn, outheight, outwidth);
end;

I1_out2=I1_out/max(max(max(I1_out)));
imwrite(I1_out2,strcat(fname,'_out.jpg'));
Nimg=I1_out2;

function IOut= undistort_image_help(ocam_model, IIn, outheight, outwidth);

width=ocam_model.width;
height=ocam_model.height;
xc=ocam_model.xc;
yc=ocam_model.yc;

zscale=75/250;

origheight=size(IIn,1);
origwidth=size(IIn,2);
scale=origheight/height;
cx=outheight/2;
cy=outwidth/2;

if length(size(IIn))==3
    RI=IIn(:,:,1);
    GI=IIn(:,:,2);
    BI=IIn(:,:,3);
else
    RI=IIn;
    GI=IIn;
    BI=IIn;
end;


[mx, my]=meshgrid(1:outheight,1:outwidth);
mx=mx';
my=my';
mr=sqrt((mx-cx).^2+(my-cy).^2);
mth=atan(mr/(zscale*outwidth));
mth(find(mth<0))=mth(find(mth<0))+pi;
mth(find(mth>pi))=mth(find(mth>pi))-pi;
%mth(find(mth>pi/2))=pi-mth(find(mth>pi/2));
mrh=polyval(ocam_model.t2r,mth);
mmx=((mx-cx)./mr).*mrh;
mmx_rnd=(round(scale*(mmx+xc)));
mmy=((my-cy)./mr).*mrh;
mmy_rnd=(round(scale*(mmy+yc)));
mm_inb_ind=find(mmx_rnd > 0 & mmy_rnd > 0 & mmx_rnd <= origheight & mmy_rnd <= origwidth);

mmx_in=mmx_rnd(mm_inb_ind);
mmy_in=mmy_rnd(mm_inb_ind);
origind=(mmy_in-1)*round(origheight)+mmx_in;

mm_inb=zeros(size(mx));
r=mm_inb;
g=mm_inb;
b=mm_inb;
mm_inb(mm_inb_ind)=150;

r(mm_inb_ind)=RI(origind);
g(mm_inb_ind)=GI(origind);
b(mm_inb_ind)=BI(origind);

IOut(:,:,1)=r;
IOut(:,:,2)=g;
IOut(:,:,3)=b;

%IOut=mmx_rnd;
