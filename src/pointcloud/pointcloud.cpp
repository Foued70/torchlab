/*
  Author  : Sonia Jin
  Date    : 27 Jan 2014
  Function: Helper file for pc/pointcloud
*/

extern "C"
{
#include <math.h>
#include <limits.h>
#define PI (3.141592653589793)
#define DIST_MAX 9999
#define DIST_MIN 1
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))
}

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/core/mat.hpp"

//using namespace cv;
//using namespace std;

extern "C"
{
double norm(double nx, double ny, double nz)
{
  return sqrt(pow(nx,2) + pow(ny,2) + pow(nz,2));
}

double distance(double nx1, double ny1, double nz1,
                double nx2, double ny2, double nz2)
{
  return norm(nx1-nx2,ny1-ny2,nz1-nz2);
}

int add3(double * sum, double * xyz1, double *xyz2)
{
  sum[0] = xyz1[0] + xyz2[0];
  sum[1] = xyz1[1] + xyz2[1];
  sum[2] = xyz1[2] + xyz2[2];
}

int subtract3(double * res, double * xyz1, double *xyz2)
{
  res[0] = xyz1[0] - xyz2[0];
  res[1] = xyz1[1] - xyz2[1];
  res[2] = xyz1[2] - xyz2[2];
}

double norm3(double* xyz)
{
  return norm(xyz[0],xyz[1],xyz[2]);
}

double distance3(double * xyz1, double * xyz2)
{
  return distance(xyz1[0],xyz1[1],xyz1[2], xyz2[0],xyz2[1],xyz2[2]);
}

int direction3(double* dir, double* xyz)
{
  double n = norm3(xyz);
  dir[0] = xyz[0]/n;
  dir[1] = xyz[1]/n;
  dir[2] = xyz[2]/n;
  return 0;
}

int convert_to_phi_theta(double* phi_theta, double* unit3vector)
{
  double x = unit3vector[0];
  double y = unit3vector[1];
  double z = unit3vector[2];
  double xy2 = sqrt(pow(x,2)+pow(y,2));
  
  phi_theta[0] = asin(z);
  if (z >= 1 || xy2 == 0)
  {
    phi_theta[1] = 0;
  }
  else
  {
    phi_theta[1] = acos(x/xy2);
    if (y < 0)
    {
      phi_theta[1] = -phi_theta[1];
    }
  }
  return 0;
}

int convert_to_unit_vector(double* unit3vector, double * phi_theta)
{
  unit3vector[2] = sin(phi_theta[0]);
  unit3vector[1] = sin(phi_theta[1])*cos(phi_theta[0]);
  unit3vector[0] = cos(phi_theta[1])*cos(phi_theta[0]);
}

int direction_phi_theta(double *phi_theta, double* xyz)
{
  double *dir;
  dir = (double *)malloc(3*sizeof(double));
  direction3(dir,xyz);
  convert_to_phi_theta(phi_theta,dir);
  free(dir);
}

int direction_between_vectors(double* dir, double* xyz1, double* xyz2)
{
  double * sub;
  sub = (double *) malloc(3*sizeof(double));
  subtract3(sub,xyz1,xyz2);
  direction3(dir,sub);
  free(sub);
}

int direction_between_vectors_phi_theta(double *phi_theta, double* xyz1, double* xyz2)
{
  double * sub;
  sub = (double *) malloc(3*sizeof(double));
  subtract3(sub,xyz1,xyz2);
  direction_phi_theta(phi_theta,sub);
  free(sub);
}

double find_d_of_plane(double x, double y, double z, double nx, double ny, double nz)
{
  return (0 - (x*nx + y*ny + z*nz));
}

double distance_to_plane(double x, double y, double z, double nx, double ny, double nz, double d)
{
  return (fabs(x*nx + y*ny + z*nz + d) / sqrt(pow(nx,2)+pow(ny,2)+pow(nz,2)));
}



int get_valid_mask(char* valid_mask, short* hwindices, long length, int height, int width)
{
  long i;
  int h,w,ind;
  i =0;
  for (i = 0; i < length; i++)
  {
    h = (int)(*(hwindices+2*i+0)-1);
    w = (int)(*(hwindices+2*i+1)-1);
    ind = (int)(h*width + w);
    *(valid_mask+ind) = 1;
  }
  
  return 0;
}


  
int get_step_maps_updown(int* step_updown, double* xyz, double min_step_size, 
                         double max_step_size, int dir, int height, int width)
{
  int h,w;
  int s;
  int sh,sw;
  double xcur,ycur,zcur;
  double xque,yque,zque;
  double dist;
  
  int offset0 = 0*height*width;
  int offset1 = 1*height*width;
  int offset2 = 2*height*width;
  
  for(h= 0; h < height; h++)
  {
    for(w = 0; w<width; w++)
    {
    
      xcur = xyz[offset0+h*width+w];
      ycur = xyz[offset1+h*width+w];
      zcur = xyz[offset2+h*width+w];
      
      if (norm(xcur,ycur,zcur) < 0.01)
      {
        step_updown[offset0+h*width+w] = h;
        step_updown[offset1+h*width+w] = w;
      }
      else
      {
        
        s = 0;
        sh = h;
        sw = w;
        dist = 0;
        
        while(h+(dir*s) >= 0 && h+(dir*s) < height)
        {
          
          sh = h+(dir*s);
          xque = xyz[offset0+sh*width+sw];
          yque = xyz[offset1+sh*width+sw];
          zque = xyz[offset2+sh*width+sw];
          
          if (norm(xque,yque,zque) < 0.01)
          {
            if (s > 0)
            {
              s = MAX(0,s-1);
              sh = h+(dir*s);;
              break;
            }
          }
          
          dist = distance(xcur,ycur,zcur,xque,yque,zque);
          
          if (dist > max_step_size)
          {
            //went over an edge
            if (s < 2)
            {
              //can't decrease s
              break;
            }
            else
            {
              s = s-1;
              sh = h+(dir*s);
              break;
            }
          }
          if (dist >= min_step_size)
          {
            //found!
            break;
          }
          
          s++;
        }
        step_updown[offset0+h*width+w] = sh;
        step_updown[offset1+h*width+w] = sw;
      }
    }
  }
}

int get_step_maps_leftright(int* step_leftright, double *xyz, double *theta, double min_step_size,
                            double max_step_size, int dir, int height, int width)
{
  int h,w;
  int s;
  int sh,sw,ph;
  double xcur,ycur,zcur,tcur;
  double xque,yque,zque,tque;
  double dist,difft;
  
  int offset0 = 0*height*width;
  int offset1 = 1*height*width;
  int offset2 = 2*height*width;
  
  for(h= 0; h < height; h++)
  {
    for(w = 0; w<width; w++)
    {
    
      xcur = xyz[offset0+h*width+w];
      ycur = xyz[offset1+h*width+w];
      zcur = xyz[offset2+h*width+w];
      
      if (norm(xcur,ycur,zcur) < 0.01)
      {
        step_leftright[offset0+h*width+w] = h;
        step_leftright[offset1+h*width+w] = w;
      }
      else
      {
      
        tcur = theta[offset0+h*width+w];
        
        s = 0;
        sh = h;
        ph = h;
        sw = w;
        dist = 0;
        difft = 0;
        
        while(s < width/3)
        {
          
          sw = (w+(dir*s)+width) % width;
          
          if (sh < 0 || sh >= height)
          {
            s = MAX(0,s-1);
            sw = (w+(dir*s)+width) % width;
            sh = ph;
            break;
          }
          
          xque = xyz[offset0+sh*width+sw];
          yque = xyz[offset1+sh*width+sw];
          zque = xyz[offset2+sh*width+sw];
          
          if (norm(xque,yque,zque) < 0.01)
          {
            if (s > 0)
            {
              s = MAX(0,s-1);
              sw = (w+(dir*s)+width) % width;
              sh = ph;
              break;
            }
          }
          
          tque = theta[offset0+sh*width+sw];
          difft = tcur-tque;
          if (difft > PI)
          {
            difft = difft-2*PI;
          }
          if (difft <= -PI)
          {
            difft = difft+2*PI;
          }
          
          if (difft*dir >= 0)
          {
          
            dist = distance(xcur,ycur,0,xque,yque,0);
          
            if (dist > max_step_size)
            {
              //went over an edge
              if(s > 1)
              {
                s = MAX(0,s-1);
                sw = (w+(dir*s)+width) % width;
                sh = ph;
              }
              break;
            }
            if (dist >= min_step_size)
            {
              //found!
              break;
            }
            ph = sh;
          }
          /*else
          {
            printf("keep looking %d %d %f %f\n",h,w,tcur,tque);
          }*/
          s++;
        }
        step_leftright[offset0+h*width+w] = sh;
        step_leftright[offset1+h*width+w] = sw;
      }
    }
  }
}


int get_step_maps(int* step_up, int* step_down, int* step_left, int* step_right, 
                  double* xyz, double* theta_map,
                  double min_step_size, double max_step_size, int height, int width)
{
  get_step_maps_updown(step_up,xyz,min_step_size,max_step_size,-1,height,width);
  get_step_maps_updown(step_down,xyz,min_step_size,max_step_size,1,height,width);
  get_step_maps_leftright(step_left,xyz,theta_map,min_step_size,max_step_size,-1,height,width);
  get_step_maps_leftright(step_right,xyz,theta_map,min_step_size,max_step_size,1,height,width);
}


int get_normal_theta_map(double* theta_map, double* centered_point_map, 
                  int* step_left, int* step_right,
                  int height, int width)
{
  int h,w, pw, nw;
  double * xyz_hw, * xyz_phw, * xyz_nhw;
  double * pt_p, * pt_n, *pt_c;
  double theta_p, theta_n, theta_c;
  double dist_p, dist_n, dist_c;
  int k;
  
  xyz_hw  = (double *) malloc(3*sizeof(double));
  xyz_phw = (double *) malloc(3*sizeof(double));
  xyz_nhw = (double *) malloc(3*sizeof(double));
  
  pt_p = (double *) malloc(2*sizeof(double));
  pt_n = (double *) malloc(2*sizeof(double));
  pt_c = (double *) malloc(2*sizeof(double));
  
  for (h = 0; h < height; h++)
  {
    for (w = 0; w < width; w++)
    {
      
      pw = step_left[height*width + h*width + w];
      nw = step_right[height*width + h*width + w];
      
      theta_c = -1000;
      
      for (k = 0; k < 3; k++)
      {
        xyz_hw[k] = centered_point_map[k*height*width+h*width+w];
        xyz_phw[k] = centered_point_map[k*height*width+h*width+pw];
        xyz_nhw[k] = centered_point_map[k*height*width+h*width+nw];
      }
      
      if (norm3(xyz_hw) > 0.01)
      {
      
        direction_between_vectors_phi_theta(pt_p, xyz_phw, xyz_hw);
        direction_between_vectors_phi_theta(pt_n, xyz_hw, xyz_nhw);
        direction_between_vectors_phi_theta(pt_c, xyz_phw, xyz_nhw);
        
        dist_p = distance3(xyz_phw,xyz_hw);
        dist_n = distance3(xyz_hw,xyz_nhw);
        dist_c = distance3(xyz_phw,xyz_nhw);
      
        theta_p = pt_p[1]-PI/2;
        theta_n = pt_n[1]-PI/2;
        
        if (norm3(xyz_nhw) > 0.01 && norm3(xyz_phw) && dist_c > 0.01)
        {
          theta_c = pt_c[1]-PI/2;
        }
        else if (norm3(xyz_nhw) > 0.01 && dist_n > 0.01)
        {
          theta_c = theta_n;
        }
        else if (norm3(xyz_phw) > 0.01 && dist_p > 0.01)
        {
          theta_c = theta_p;
        }
        
        if (theta_c <= -PI && theta_c != -1000)
        {
          theta_c = theta_c + 2*PI;
        }
      }
      
      theta_map[h*width+w] = theta_c;
      
    }
  }
  
  free(xyz_hw);
  free(xyz_phw);
  free(xyz_nhw);
  
  free(pt_p);
  free(pt_n);
}

int get_normal_phi_map(double* phi_map, double* centered_point_map, 
                int* step_up, int* step_down,
                int height, int width)
{
  int h,w, ph, nh;
  double * xyz_hw, * xyz_phw, * xyz_nhw;
  double phi_p, phi_n, phi_c;
  double dia_p, dia_c, dia_n;
  double dz_p, dz_n, dz_c, dd_p, dd_n, dd_c;
  double rad_p,rad_n,rad_c;
  double dist_p, dist_n, dist_c;
  int k;
  
  xyz_hw  = (double *) malloc(3*sizeof(double));
  xyz_phw = (double *) malloc(3*sizeof(double));
  xyz_nhw = (double *) malloc(3*sizeof(double));
  
  
  for (w = 0; w < width; w++)
  {
    for (h = 0; h < height; h++)
    {
      
      ph = step_up[h*width + w];
      nh = step_down[h*width + h];
      
      phi_c = -1000;
      
      for (k = 0; k < 3; k++)
      {
        xyz_hw[k] = centered_point_map[k*height*width+h*width+w];
        xyz_phw[k] = centered_point_map[k*height*width+ph*width+w];
        xyz_nhw[k] = centered_point_map[k*height*width+nh*width+w];
      }
      
      if (norm3(xyz_hw) > 0.01)
      {
      
        dist_p = distance3(xyz_phw,xyz_hw);
        dist_n = distance3(xyz_hw,xyz_nhw);
        dist_c = distance3(xyz_phw,xyz_nhw);
      
        dia_p = sqrt(pow(xyz_phw[0],2) + pow(xyz_phw[1],2));
        dia_c = sqrt(pow(xyz_hw[0],2) + pow(xyz_hw[1],2));
        dia_n = sqrt(pow(xyz_nhw[0],2) + pow(xyz_nhw[1],2));
      
        dd_p = dia_p - dia_c;
        dd_n = dia_c - dia_n;
        dd_c = dia_p - dia_n;
      
        dz_p = xyz_phw[2]-xyz_hw[2];
        dz_n = xyz_hw[2]-xyz_nhw[2];
        dz_c = xyz_phw[2]-xyz_nhw[2];
        
        rad_p = sqrt(pow(dd_p,2)+pow(dz_p,2));
        rad_n = sqrt(pow(dd_n,2)+pow(dz_n,2));
        rad_c = sqrt(pow(dd_c,2)+pow(dz_c,2));
      
        phi_p = -asin(dd_p/rad_p);
        phi_n = -asin(dd_n/rad_n);
      
        if (norm3(xyz_nhw) > 0.01 && norm3(xyz_phw) && h > 0 && nh < height && dist_c > 0.01 && rad_c > 0)
        {
          phi_c = -asin(dd_c/rad_c);
        }
        else if (norm3(xyz_nhw) > 0.01 && nh < height && dist_n > 0.01 && rad_n > 0)
        {
          phi_c = -phi_n;
        }
        else if (norm3(xyz_phw) > 0.01 && h > 0 && dist_p > 0.01 && rad_p > 0)
        {
          phi_c = -phi_p;
        }
      }
      
      phi_map[h*width+w] = phi_c;
      
    }
  }
  
  free(xyz_hw);
  free(xyz_phw);
  free(xyz_nhw);
  
}


int increment_pixel_in_image(double* img, double h, double w, int height, int width, double incr)
{
  int hh = (int)(round(h));
  int ww = (int)(round(w));
  
  if ( (hh < height) && (ww < width) &&
       (hh >= 0) && (ww >= 0) )
  {
    *(img+hh*width+ww) = *(img+hh*width+ww) + incr;
  }
  return 0;
}

int connect_lines_in_image(double* img, double y1, double x1, double y2, double x2, 
                           int height, int width, double incr)
{

  double minx,maxx,miny,maxy,x,y;
  double xx,yy,prev;
  double dffx,dffy,slp;
  
  minx = fmin(x1,x2);
  maxx = fmax(x1,x2);
  miny = fmin(y1,y2);
  maxy = fmax(y1,y2);
    
  dffx = x1-x2;
  dffy = y1-y2;
  
  if (minx == maxx)
  {
    increment_pixel_in_image(img, (int)y1, (int)x1, height, width, incr/2);
  }
  else
  { 
    prev = 0;
    if (fabs(dffx) >= fabs(dffy))
    {
      slp = dffy/dffx;
      for (x = minx; x <= maxx; x++)
      {
        xx = round(x);
        yy = round(y2 + (x-x2)*slp);
        
        increment_pixel_in_image(img, yy, xx, height, width, incr);
        if ((prev > 0) && (yy != prev))
        {
          increment_pixel_in_image(img, prev, xx, height, width, incr/2);
          increment_pixel_in_image(img, yy, xx-1, height, width, incr/2);
        }
        prev = yy;
      }
    
    }
    else
    {
      slp = dffx/dffy;
      for (y = miny; y <= maxy; y++)
      {
        yy = round(y);
        xx = round(x2 + (y-y2)*slp);
        increment_pixel_in_image(img, yy, xx, height, width, incr);
        if ((prev > 0) && (xx != prev))
        {
          increment_pixel_in_image(img, yy, prev, height, width, incr/2);
          increment_pixel_in_image(img, yy-1, xx, height, width, incr/2);
        }
        prev = xx;
      }
    }
  }
  return 0;
}

int flatten_image(double* imagez, double* image_corner,
                                          double* coord_map, double *points,
                                          char* connection_map, char* corners_map,
                                          double* corners_map_filled,
                                          int pan_hght, int pan_wdth,
                                          int img_hght, int img_wdth)
{

  int h,w, pw;
  double crdx,crdy, pcrdx,pcrdy;
  double cor_x,cor_y;
  double ptx,pty,ptz, ptxp,ptyp,ptzp;
  
  char conn_hw, conn_hpw;
  char corn_hw;
  
  char on_corner;
  
  double dst_con, dst_cor;
  double cor_stt, cor_end;
  
  double slp1,slp2;
  
  for(w = 0; w < pan_wdth; w++)
  {
  
    pw = (w - 1 + pan_wdth) % pan_wdth;
    
    dst_con = 0;
    dst_cor = 0;

    cor_stt = 0;
    cor_end = 0;    
    cor_x = 0;
    cor_y = 0;
    on_corner = 0;
    
    for (h = 0; h < pan_hght; h++)
    {
      /* find it on connection_map and corner_map */
      conn_hw = *(connection_map + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      conn_hpw = *(connection_map + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      corn_hw = *(corners_map + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      
      crdy = *(coord_map + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      crdx = *(coord_map + 1*pan_hght*pan_wdth + h*pan_wdth + w);
      
      pcrdy = *(coord_map + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      pcrdx = *(coord_map + 1*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      ptx = *(points + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      pty = *(points + 1*pan_hght*pan_wdth + h*pan_wdth + w);
      ptz = *(points + 2*pan_hght*pan_wdth + h*pan_wdth + w);
      
      ptxp = *(points + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      ptyp = *(points + 1*pan_hght*pan_wdth + h*pan_wdth + pw);
      ptzp = *(points + 2*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      /* check corner */
      if (on_corner == 0)
      {
        /* starting new corner column */
        if (corn_hw == 1)
        {
          cor_stt = ptz;
          cor_end = ptz;
          cor_y = crdy;
          cor_x = crdx;
          on_corner = 1;
          *(corners_map_filled+0*pan_hght*pan_wdth + h*pan_wdth + w) = 1;
        }
      }
      else
      {
        /* continue existing column */
        if ((corn_hw == 1) && (crdx == cor_x) && (crdy == cor_y))
        {
          cor_end = ptz;
          on_corner = 1;
          *(corners_map_filled+0*pan_hght*pan_wdth + h*pan_wdth + w) = 1;
        }
        else
        {
          /* stop and draw */
          dst_cor = 1 + 100*fabs(cor_stt - cor_end);
          increment_pixel_in_image(image_corner, cor_y, cor_x, img_hght, img_wdth, dst_cor);
        
          /* maybe start new column, otherwise refresh */
          if (corn_hw == 1)
          {
            cor_stt = ptz;
            cor_end = ptz;
            cor_y = crdy;
            cor_x = crdx;
            on_corner = 1;
            *(corners_map_filled+0*pan_hght*pan_wdth + h*pan_wdth + w) = 1;
          }
          else
          {
            cor_stt = 0;
            cor_end = 0;
            cor_y = 0;
            cor_x = 0;
            on_corner = 0;
          }
          
        }
      }
      
      /* check connection */
      if (conn_hw == 1)
      {
        dst_con = norm(ptx,pty,0) + 1;
        
        if (conn_hpw == 1)
        {
          connect_lines_in_image(imagez, crdy, crdx, pcrdy, pcrdx, img_hght, img_wdth, dst_con);
        }
        else
        {
          increment_pixel_in_image(imagez, crdy, crdx, img_hght, img_wdth, dst_con);
        }
      }
      
    }
  }
  
  return 0;
}

int flatten_image_with_theta(double* imagez, double* image_corner, double* corners_map_filled,
                                          double* imaget, double* theta_cnt,
                                          double* coord_map, double *points,
                                          char* connection_map, char* corners_map, double* theta,
                                          int pan_hght, int pan_wdth,
                                          int img_hght, int img_wdth)
{

  int h,w, pw;
  double crdx,crdy, pcrdx,pcrdy;
  double cor_x,cor_y;
  double ptx,pty,ptz, ptxp,ptyp,ptzp;
  double theta_p;
  
  char conn_hw, conn_hpw;
  char corn_hw;
  
  char on_corner;
  
  double dst_con, dst_cor;
  double cor_stt, cor_end;
  
  double slp1,slp2;
  
  for(w = 0; w < pan_wdth; w++)
  {
  
    pw = (w - 1 + pan_wdth) % pan_wdth;
    
    dst_con = 0;
    dst_cor = 0;

    cor_stt = 0;
    cor_end = 0;    
    cor_x = 0;
    cor_y = 0;
    on_corner = 0;
    
    for (h = 0; h < pan_hght; h++)
    {
      /* find it on connection_map and corner_map */
      conn_hw = *(connection_map + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      conn_hpw = *(connection_map + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      corn_hw = *(corners_map + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      
      crdy = *(coord_map + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      crdx = *(coord_map + 1*pan_hght*pan_wdth + h*pan_wdth + w);
      
      pcrdy = *(coord_map + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      pcrdx = *(coord_map + 1*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      ptx = *(points + 0*pan_hght*pan_wdth + h*pan_wdth + w);
      pty = *(points + 1*pan_hght*pan_wdth + h*pan_wdth + w);
      ptz = *(points + 2*pan_hght*pan_wdth + h*pan_wdth + w);
      
      ptxp = *(points + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      ptyp = *(points + 1*pan_hght*pan_wdth + h*pan_wdth + pw);
      ptzp = *(points + 2*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      theta_p = *(theta + 0*pan_hght*pan_wdth + h*pan_wdth + pw);
      
      /* check corner */
      if (on_corner == 0)
      {
        /* starting new corner column */
        if (corn_hw == 1)
        {
          cor_stt = ptz;
          cor_end = ptz;
          cor_y = crdy;
          cor_x = crdx;
          on_corner = 1;
          *(corners_map_filled+0*pan_hght*pan_wdth + h*pan_wdth + w) = 1;
          increment_pixel_in_image(imaget, crdy, crdx, img_hght, img_wdth, theta_p);
          increment_pixel_in_image(theta_cnt, crdy, crdx, img_hght, img_wdth, 1);
        }
      }
      else
      {
        /* continue existing column */
        if ((corn_hw == 1) && (crdx == cor_x) && (crdy == cor_y))
        {
          cor_end = ptz;
          on_corner = 1;
          *(corners_map_filled+0*pan_hght*pan_wdth + h*pan_wdth + w) = 1;
          increment_pixel_in_image(imaget, crdy, crdx, img_hght, img_wdth, theta_p);
          increment_pixel_in_image(theta_cnt, crdy, crdx, img_hght, img_wdth, 1);
        }
        else
        {
          /* stop and draw */
          dst_cor = 1 + 100*fabs(cor_stt - cor_end);
          increment_pixel_in_image(image_corner, cor_y, cor_x, img_hght, img_wdth, dst_cor);
        
          /* maybe start new column, otherwise refresh */
          if (corn_hw == 1)
          {
            cor_stt = ptz;
            cor_end = ptz;
            cor_y = crdy;
            cor_x = crdx;
            on_corner = 1;
            *(corners_map_filled+0*pan_hght*pan_wdth + h*pan_wdth + w) = 1;
            increment_pixel_in_image(imaget, crdy, crdx, img_hght, img_wdth, theta_p);
            increment_pixel_in_image(theta_cnt, crdy, crdx, img_hght, img_wdth, 1);
          }
          else
          {
            cor_stt = 0;
            cor_end = 0;
            cor_y = 0;
            cor_x = 0;
            on_corner = 0;
          }
          
        }
      }
      
      /* check connection */
      if (conn_hw == 1)
      {
        dst_con = norm(ptx,pty,0) + 1;
        
        if (conn_hpw == 1)
        {
          connect_lines_in_image(imagez, crdy, crdx, pcrdy, pcrdx, img_hght, img_wdth, dst_con);
          connect_lines_in_image(imaget, crdy, crdx, pcrdy, pcrdx, img_hght, img_wdth, theta_p);
          connect_lines_in_image(theta_cnt, crdy, crdx, pcrdy, pcrdx, img_hght, img_wdth, 1);
        }
        else
        {
          increment_pixel_in_image(imagez, crdy, crdx, img_hght, img_wdth, dst_con);
          increment_pixel_in_image(imaget, crdy, crdx, img_hght, img_wdth, theta_p);
          increment_pixel_in_image(theta_cnt, crdy, crdx, img_hght, img_wdth, 1);
        }
      }
      
    }
  }
  
  return 0;
}

}