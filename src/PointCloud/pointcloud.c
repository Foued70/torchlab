/*
 * pointcloud.c
 *
 *  Created on: Sep. 30, 2013
 *      Author: Sonia Jin
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#define PI (3.141592653589793)
#define DIST_MAX 9999
#define DIST_MIN 1
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))


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
  dir = malloc(3*sizeof(double));
  direction3(dir,xyz);
  convert_to_phi_theta(phi_theta,dir);
  free(dir);
}

int direction_between_vectors(double* dir, double* xyz1, double* xyz2)
{
  double * sub;
  sub = malloc(3*sizeof(double));
  subtract3(sub,xyz1,xyz2);
  direction3(dir,sub);
  free(sub);
}

int direction_between_vectors_phi_theta(double *phi_theta, double* xyz1, double* xyz2)
{
  double * sub;
  sub = malloc(3*sizeof(double));
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

int get_index_and_mask(long* index_map, short* mask_map, double* points, short* hwindices,
                       long length, int height, int width)
{
  long i,k;
  int h,w,ind;
  i =0;
  for (i = 0; i < length; i++)
  {
    h = (int)(*(hwindices+2*i+0)-1);
    w = (int)(*(hwindices+2*i+1)-1);
    ind = (int)(h*width + w);
    *(index_map+ind) = (long)(i+1);
    *(mask_map+ind) = (int)(0);
  }
  
  return 0;
}

int downsample_with_widthheightinfo(double* downsampled_points, double* downsampled_rgb, 
                             int* downsampled_count, double* coord_map, 
                             double* points_map, double* rgb_map, 
                             int height, int width)
{

  int w, h, pw, ph, dcount = 0;
  int indx, indy, indz, 
      indx_hpw, indy_hpw, indz_hpw, 
      indx_phw, indy_phw, indz_phw,
      indx_phpw, indy_phpw, indz_phpw;
  double crdx, crdy, crdz,
         crdx_hpw, crdy_hpw, crdz_hpw,
         crdx_phw, crdy_phw, crdz_phw,
         crdx_phpw, crdy_phpw, crdz_phpw;
  double ptx,pty,ptz;
  
  
  for (h = 0; h < height; h++)
  {
    ph = (int)fmax(0,h-1);
    for (w = 0; w < width; w++)
    {
      pw = (w-1+width) % width;
      
      indx = 0*height*width + h*width+w;
      indy = 1*height*width + h*width+w;
      indz = 2*height*width + h*width+w;
      
      crdx = *(coord_map + indx);
      crdy = *(coord_map + indy);
      crdz = *(coord_map + indz);
      
      indx_hpw = 0*height*width + h*width+pw;
      indy_hpw = 1*height*width + h*width+pw;
      indz_hpw = 2*height*width + h*width+pw;
      
      crdx_hpw = *(coord_map + indx_hpw);
      crdy_hpw = *(coord_map + indy_hpw);
      crdz_hpw = *(coord_map + indz_hpw);
      
      indx_phw = 0*height*width + ph*width+w;
      indy_phw = 1*height*width + ph*width+w;
      indz_phw = 2*height*width + ph*width+w;
      
      crdx_phw = *(coord_map + indx_phw);
      crdy_phw = *(coord_map + indy_phw);
      crdz_phw = *(coord_map + indz_phw);
      
      indx_phpw = 0*height*width + ph*width+pw;
      indy_phpw = 1*height*width + ph*width+pw;
      indz_phpw = 2*height*width + ph*width+pw;
      
      crdx_phpw = *(coord_map + indx_phpw);
      crdy_phpw = *(coord_map + indy_phpw);
      crdz_phpw = *(coord_map + indz_phpw);
      
      if ((distance(crdx,crdy,crdz,crdx_hpw,crdy_hpw, crdz_hpw) > 0) &&
          ((ph == h) ||
           ((distance(crdx,crdy,crdz,crdx_phw,crdy_phw, crdz_phw) > 0) &&
            (distance(crdx,crdy,crdz,crdx_phw,crdy_phw, crdz_phw) > 0))))
      {
        
        ptx = *(points_map +indx); 
        pty = *(points_map +indy); 
        ptz = *(points_map +indz); 
        
        if ((norm(ptx,pty,ptz) > DIST_MIN) && (norm(ptx,pty,ptz) < DIST_MAX))
        {
        
          *(downsampled_points+3*dcount+0) = ptx; 
          *(downsampled_points+3*dcount+1) = pty; 
          *(downsampled_points+3*dcount+2) = ptz; 
        
          *(downsampled_rgb+3*dcount+0) = *(rgb_map +indx); 
          *(downsampled_rgb+3*dcount+1) = *(rgb_map +indy); 
          *(downsampled_rgb+3*dcount+2) = *(rgb_map +indz);
          
          dcount ++;
          
        }
        
      }
      
    }
  }
  
  (*downsampled_count) = dcount;
  
  return 0;
}

int downsample_without_widthheightinfo(double* downsampled_points, double* downsampled_rgb, 
                                int* downsampled_count, int* coord_list, 
                                double* points_list, double* rgb_list, 
                                int length, int binx, int biny, int binz)
{
  int* bins;
  int i, dcount = 0;
  int cx, cy, cz, b;
  
  bins = malloc(binx*biny*binz*sizeof(int));
  
  printf("bins created\n");
  
  for (i = 0; i < length; i++)
  {
    cx = coord_list[3*i+0];
    cy = coord_list[3*i+1];
    cz = coord_list[3*i+2];
    
    b = bins[cx * biny * binz + cy * binz + cz];
    
    if (b == 0)
    {
      downsampled_points[3*dcount+0] = points_list[3*i+0];
      downsampled_points[3*dcount+1] = points_list[3*i+1];
      downsampled_points[3*dcount+2] = points_list[3*i+2];
      
      downsampled_rgb[3*dcount+0] = rgb_list[3*i+0];
      downsampled_rgb[3*dcount+1] = rgb_list[3*i+1];
      downsampled_rgb[3*dcount+2] = rgb_list[3*i+2];
      
      bins[cx * biny * binz + cy * binz + cz] = 1;
      
      dcount ++;
    }
    
  }
  
  (*downsampled_count) = dcount;
  free(bins);
  
  return 0;
  
}

int make_normal_list(double* normal_list, double* normal_map, 
                     int* hwindices, int length, int height, int width)
{
  int i;
  int h, w;
  int indx, indy, indz;
  double nx, ny, nz;
  
  for (i = 0; i < length; i++)
  {
    h = *(hwindices+2*i+0)-1;
    w = *(hwindices+2*i+1)-1;
    
    indx = 0*height*width + h*width + w;
    indy = 1*height*width + h*width + w;
    indz = 2*height*width + h*width + w;
    
    nx = *(normal_map+indx);
    ny = *(normal_map+indy);
    nz = *(normal_map+indz);
    
    *(normal_list+3*i+0) = nx;
    *(normal_list+3*i+1) = ny;
    *(normal_list+3*i+2) = nz;
  }
  
  return 0;
}

int flatten_image_without_widthheightinfo(double* imagez, int* coords, double* dists,
                                              int length, int height, int width)
{

  int h,w,i,ind;
  
  for (i = 0; i<length; i++)
  {
    h = *(coords + 2*i + 0);
    w = *(coords + 2*i + 1);
    ind = h*width + w;
    *(imagez+ind) = *(imagez+ind) + *(dists+i);
  } 
  
  return 0;
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

int flatten_image_with_widthheightinfo(double* imagez, double* image_corner,
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
  double thresh = 0.1;
  
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

int theta_map(double* theta_map, double* centered_point_map, int height, int width)
{
  int h,w, pw, nw;
  double * xyz_hw, * xyz_phw, * xyz_nhw;
  double * pt_p, * pt_n, *pt_c;
  double theta_p, theta_n, theta_c;
  double dist_p, dist_n, dist_c;
  int k;
  
  xyz_hw = malloc(3*sizeof(double));
  xyz_phw = malloc(3*sizeof(double));
  xyz_nhw = malloc(3*sizeof(double));
  
  pt_p = malloc(2*sizeof(double));
  pt_n = malloc(2*sizeof(double));
  pt_c = malloc(2*sizeof(double));
  
  for (h = 0; h < height; h++)
  {
    for (w = 0; w < width; w++)
    {
      
      pw = MAX(0,w-1);
      nw = MIN(width-1,w+1);
      
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

int phi_map(double* phi_map, double* centered_point_map, int height, int width)
{
  int h,w, ph, nh;
  double * xyz_hw, * xyz_phw, * xyz_nhw;
  double phi_p, phi_n, phi_c;
  double dia_p, dia_c, dia_n;
  double dz_p, dz_n, dz_c, dd_p, dd_n, dd_c;
  double rad_p,rad_n,rad_c;
  double dist_p, dist_n, dist_c;
  int k;
  
  xyz_hw = malloc(3*sizeof(double));
  xyz_phw = malloc(3*sizeof(double));
  xyz_nhw = malloc(3*sizeof(double));
  
  
  for (w = 0; w < width; w++)
  {
    for (h = 0; h < height; h++)
    {
      ph = MAX(0,h-1);
      nh = MIN(height-1,h+1);
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

int theta_map_smooth(double* smoothed_theta_map, double* theta_map, char * extant_map, 
                     int height, int width, int max_win, double max_theta_diff)
{
  int h,w;
  int hh,ww;
  int i,j;
  int index_curr, index_query;
  int step;
  int stop = 0;
  double theta_curr, theta_query, theta_diff;
  double theta_sum, theta_count, theta_mean;
  char extant_curr, extant_query;
  
  for (h = 0; h < height; h++)
  {
    for (w = 0; w < width; w++)
    {
    
      index_curr = h*width + w;
      extant_curr = extant_map[index_curr];
      theta_curr = theta_map[index_curr];
      theta_mean = theta_curr;
      
      if (extant_curr == 1)
      {      
        theta_mean = 0;
        theta_sum = 0;
        theta_count = 0;
        
        for (i = -1; i <= 1; i++)
        {
          for( j = -1; j <=1; j++)
          {
            step = 0;
            stop = 0;
            
            do
            {
            
              step ++;
              hh = h+i*step;
              ww = w+j*step;
            
              if (hh < 0 || hh >= height || ww < 0 || ww >= width)
              {
                stop = 1;
              }
              else
              {
                index_query = hh*width + ww;
                extant_query = extant_map[index_query];
                
                if (extant_query != 1)
                {
                  stop = 1;
                }
                else
                {
                  theta_query = theta_map[index_query];
                  theta_diff = fabs(theta_query-theta_curr);
                  
                  if (theta_diff <= max_theta_diff)
                  {
                    theta_sum = theta_sum + theta_query;
                    theta_count++;
                  }
                  else
                  {
                    theta_diff = fabs(theta_query + 2*PI -theta_curr);
                    if (theta_diff <= max_theta_diff)
                    {
                      theta_sum = theta_sum + theta_query + 2*PI;
                      theta_count++;
                    }
                    else
                    {
                      theta_diff = fabs(theta_query - 2*PI -theta_curr); 
                      if (theta_diff <= max_theta_diff)
                      {
                        theta_sum = theta_sum + theta_query - 2*PI;
                        theta_count++;
                      }
                      else
                      {
                        stop = 1;
                      }
                    }
                  }
                  
                  if (i == 0 && j == 0)
                  {
                    stop = 1;
                  }
                  else if (step >= max_win)
                  {
                    stop = 1;
                  }
                }
                
              }
            } while(stop != 1);
            
          }
        }
        
        if (theta_count == 0)
        {
          theta_mean = theta_curr;
        }
        else
        {
          theta_mean = theta_sum/theta_count;
          
          if (theta_mean > PI)
          {
            theta_mean = theta_mean - 2*PI;
          }
          else if (theta_mean <= -PI)
          {
            theta_mean = theta_mean + 2*PI;
          }
        }
      }
      
      smoothed_theta_map[index_curr] = theta_mean;
    }
  }
}

int phi_map_smooth(double* smoothed_phi_map, double* phi_map, char * extant_map, 
                     int height, int width, int max_win, double max_phi_diff)
{
  int h,w;
  int hh,ww;
  int i,j;
  int index_curr, index_query;
  int step;
  int stop = 0;
  double phi_curr, phi_query, phi_diff;
  double phi_sum, phi_count, phi_mean;
  char extant_curr, extant_query;
  
  for (h = 0; h < height; h++)
  {
    for (w = 0; w < width; w++)
    {
    
      index_curr = h*width + w;
      extant_curr = extant_map[index_curr];
      phi_curr = phi_map[index_curr];
      phi_mean = phi_curr;
      
      if (extant_curr == 1)
      {      
        phi_mean = 0;
        phi_sum = 0;
        phi_count = 0;
        
        for (i = -1; i <= 1; i++)
        {
          for( j = -1; j <=1; j++)
          {
            step = 0;
            stop = 0;
            
            do
            {
            
              step ++;
              hh = h+i*step;
              ww = w+j*step;
            
              if (hh < 0 || hh >= height || ww < 0 || ww >= width)
              {
                stop = 1;
              }
              else
              {
                index_query = hh*width + ww;
                extant_query = extant_map[index_query];
                
                if (extant_query != 1)
                {
                  stop = 1;
                }
                else
                {
                  phi_query = phi_map[index_query];
                  phi_diff = fabs(phi_curr - phi_query);
                  
                  if (phi_diff > max_phi_diff)
                  {
                    stop = 1;
                  }
                  else
                  {
                  
                    phi_sum = phi_sum + phi_query;
                    phi_count++;
                    
                    if (i == 0 && j == 0)
                    {
                      stop = 1;
                    }
                    else if (step >= max_win)
                    {
                      stop = 1;
                    }
                  }
                }
                
              }
            } while(stop != 1);
            
          }
        }
        
        if (phi_count == 0)
        {
          phi_mean = phi_curr;
        }
        else
        {
          phi_mean = phi_sum/phi_count;
        }
      }
      
      smoothed_phi_map[index_curr] = phi_mean;
    }
  }
}

/*
static double PYTHAG(double a, double b)
{
    double at = fabs(a), bt = fabs(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
    else if (bt > 0.0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
    else result = 0.0;
    return(result);
}

int dsvd(double *a, int m, int n, double *w, double *v)
{
    int flag, i, its, j, jj, k, l, nm;
    double c, f, h, s, x, y, z;
    double anorm = 0.0, g = 0.0, scale = 0.0;
    double *rv1;
  
    if (m < n) 
    {
        fprintf(stderr, "#rows must be > #cols \n");
        return(0);
    }
  
    rv1 = (double *)malloc((unsigned int) n*sizeof(double));

    //Householder reduction to bidiagonal form
    for (i = 0; i < n; i++) 
    {
        // left-hand reduction
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m) 
        {
            for (k = i; k < m; k++) 
                scale += fabs((double)a[k*n+i]);
            if (scale) 
            {
                for (k = i; k < m; k++) 
                {
                    a[k*n+i] = (double)((double)a[k*n+i]/scale);
                    s += ((double)a[k*n+i] * (double)a[k*n+i]);
                }
                f = (double)a[i*n+i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i*n+i] = (double)(f - g);
                if (i != n - 1) 
                {
                    for (j = l; j < n; j++) 
                    {
                        for (s = 0.0, k = i; k < m; k++) 
                            s += ((double)a[k*n+i] * (double)a[k*n+j]);
                        f = s / h;
                        for (k = i; k < m; k++) 
                            a[k*n+j] += (double)(f * (double)a[k*n+i]);
                    }
                }
                for (k = i; k < m; k++) 
                    a[k*n+i] = (double)((double)a[k*n+i]*scale);
            }
        }
        w[i] = (double)(scale * g);
    
        // right-hand reduction
        g = s = scale = 0.0;
        if (i < m && i != n - 1) 
        {
            for (k = l; k < n; k++) 
                scale += fabs((double)a[i*n+k]);
            if (scale) 
            {
                for (k = l; k < n; k++) 
                {
                    a[i*n+k] = (double)((double)a[i*n+k]/scale);
                    s += ((double)a[i*n+k] * (double)a[i*n+k]);
                }
                f = (double)a[i*n+l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i*n+l] = (double)(f - g);
                for (k = l; k < n; k++) 
                    rv1[k] = (double)a[i*n+k] / h;
                if (i != m - 1) 
                {
                    for (j = l; j < m; j++) 
                    {
                        for (s = 0.0, k = l; k < n; k++) 
                            s += ((double)a[j*n+k] * (double)a[i*n+k]);
                        for (k = l; k < n; k++) 
                            a[j*n+k] += (double)(s * rv1[k]);
                    }
                }
                for (k = l; k < n; k++) 
                    a[i*n+k] = (double)((double)a[i*n+k]*scale);
            }
        }
        anorm = MAX(anorm, (fabs((double)w[i]) + fabs(rv1[i])));
    }
  
    // accumulate the right-hand transformation 
    for (i = n - 1; i >= 0; i--) 
    {
        if (i < n - 1) 
        {
            if (g) 
            {
                for (j = l; j < n; j++)
                    v[j*n+i] = (double)(((double)a[i*n+j] / (double)a[i*n+l]) / g);
                    // double division to avoid underflow
                for (j = l; j < n; j++) 
                {
                    for (s = 0.0, k = l; k < n; k++) 
                        s += ((double)a[i*n+k] * (double)v[k*n+j]);
                    for (k = l; k < n; k++) 
                        v[k*n+j] += (double)(s * (double)v[k*n+i]);
                }
            }
            for (j = l; j < n; j++) 
                v[i*n+j] = v[j*n+i] = 0.0;
        }
        v[i*n+i] = 1.0;
        g = rv1[i];
        l = i;
    }
  
    // accumulate the left-hand transformation 
    for (i = n - 1; i >= 0; i--) 
    {
        l = i + 1;
        g = (double)w[i];
        if (i < n - 1) 
            for (j = l; j < n; j++) 
                a[i*n+j] = 0.0;
        if (g) 
        {
            g = 1.0 / g;
            if (i != n - 1) 
            {
                for (j = l; j < n; j++) 
                {
                    for (s = 0.0, k = l; k < m; k++) 
                        s += ((double)a[k*n+i] * (double)a[k*n+j]);
                    f = (s / (double)a[i*n+i]) * g;
                    for (k = i; k < m; k++) 
                        a[k*n+j] += (double)(f * (double)a[k*n+i]);
                }
            }
            for (j = i; j < m; j++) 
                a[j*n+i] = (double)((double)a[j*n+i]*g);
        }
        else 
        {
            for (j = i; j < m; j++) 
                a[j*n+i] = 0.0;
        }
        ++a[i*n+i];
    }

    // diagonalize the bidiagonal form 
    for (k = n - 1; k >= 0; k--) 
    {                             // loop over singular values 
        for (its = 0; its < 30; its++) 
        {                         // loop over allowed iterations 
            flag = 1;
            for (l = k; l >= 0; l--) 
            {                     // test for splitting 
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm) 
                {
                    flag = 0;
                    break;
                }
                if (fabs((double)w[nm]) + anorm == anorm) 
                    break;
            }
            if (flag) 
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++) 
                {
                    f = s * rv1[i];
                    if (fabs(f) + anorm != anorm) 
                    {
                        g = (double)w[i];
                        h = PYTHAG(f, g);
                        w[i] = (double)h; 
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++) 
                        {
                            y = (double)a[j*n+nm];
                            z = (double)a[j*n+i];
                            a[j*n+nm] = (double)(y * c + z * s);
                            a[j*n+i] = (double)(z * c - y * s);
                        }
                    }
                }
            }
            z = (double)w[k];
            if (l == k) 
            {                  // convergence 
                if (z < 0.0) 
                {              // make singular value nonnegative 
                    w[k] = (double)(-z);
                    for (j = 0; j < n; j++) 
                        v[j*n+k] = (-v[j*n+k]);
                }
                break;
            }
            if (its >= 30) {
                free((void*) rv1);
                fprintf(stderr, "No convergence after 30,000! iterations \n");
                return(0);
            }
    
            // shift from bottom 2 x 2 minor 
            x = (double)w[l];
            nm = k - 1;
            y = (double)w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAG(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
          
            // next QR transformation 
            c = s = 1.0;
            for (j = l; j <= nm; j++) 
            {
                i = j + 1;
                g = rv1[i];
                y = (double)w[i];
                h = s * g;
                g = c * g;
                z = PYTHAG(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++) 
                {
                    x = (double)v[jj*n+j];
                    z = (double)v[jj*n+i];
                    v[jj*n+j] = (double)(x * c + z * s);
                    v[jj*n+i] = (double)(z * c - x * s);
                }
                z = PYTHAG(f, h);
                w[j] = (double)z;
                if (z) 
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++) 
                {
                    y = (double)a[jj*n+j];
                    z = (double)a[jj*n+i];
                    a[jj*n+j] = (double)(y * c + z * s);
                    a[jj*n+i] = (double)(z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = (double)x;
        }
    }
    free((void*) rv1);
    return(1);
}

int normal_3x3(double *normal, double *a)
{
  int m = 3;
  int n = 3;
  int index;
  double *w;
  double *v;
  
  w = malloc(3*sizeof(double));
  v = malloc(3*3*sizeof(double));
  
  dsvd(a,m,n,w,v);
  
  if (w[0] <= w[1] && w[0] <= w[2])
  {
    index = 0;
  }
  else if (w[1] <= w[0] && w[1] <= w[2])
  {
    index = 1;
  }
  else
  {
    index = 2;
  }
  
  normal[0] = a[0*3 + index];
  normal[1] = a[1*3 + index];
  normal[2] = a[2*3 + index];
  
  free(w);
  free(v);
  
  return 0;
}

int normal_3x3_with_point(double *normal, double *point, double *a)
{
  double d;
  
  normal_3x3(normal,a);
  
  d = point[0]*normal[0] + point[1]*normal[1] + point[2]*normal[2];
  
  if (d < 0){
    normal[0] = -normal[0];
    normal[1] = -normal[1];
    normal[2] = -normal[2];
  }
  
  return 0;
}

int mode_normal_3x3(double * best_normal, double *normals, int len, double thresh)
{

  int winsize;
  double best_x,best_y,best_z, curr_x, curr_y, curr_z, sum_x, sum_y, sum_z;
  int bin_x, bin_y, bin_z;
  int best_match = 0, curr_match = 0, factor = 0;
  double nn;
  
  int i,j,k,c;
  
  if (thresh > 1.0 || thresh <= 0)
  {
    thresh = 0.5;
  }
  
  winsize = (int)(2/thresh)+1;
  thresh = 2/(double)(winsize-1);
  
  for(i = 0; i<winsize; i++)
  {
    for(j=0; j<winsize; j++)
    {
      for(k=0; k<winsize; k++)
      {
        curr_match = 0;
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;
        
        for(c=0; c<len; c++)
        {
          curr_x = normals[c*3+0];
          curr_y = normals[c*3+1];
          curr_z = normals[c*3+2];
          
          if(norm(curr_x,curr_y,curr_z) > 0.99 && norm(curr_x,curr_y,curr_z) < 1.01)
          {
          
            bin_x = (int)((curr_x+1)/thresh);
            bin_y = (int)((curr_y+1)/thresh);
            bin_z = (int)((curr_z+1)/thresh);
          
            if (bin_x == i && bin_y == j && bin_z == k)
            {
              curr_match = curr_match + 1;
              sum_x = sum_x+curr_x;
              sum_y = sum_y+curr_y;
              sum_z = sum_z+curr_z;
            }
          }
        }
        
        if (curr_match > best_match)
        {
          best_match = curr_match;
          factor = best_match;
          best_x = sum_x;
          best_y = sum_y;
          best_z = sum_z;
        }
        else if (curr_match == best_match && curr_match > 0)
        {
          factor = factor + curr_match;
          best_x = best_x + sum_x;
          best_y = best_y + sum_y;
          best_z = best_z + sum_z;
        }
      }
    }
  }
  
  nn = norm(best_x,best_y,best_z);
  best_normal[0] = best_x/nn;
  best_normal[1] = best_y/nn;
  best_normal[2] = best_z/nn;
  
  return 0;
}

int normal_for_point_3x3(double * best_normal, double * curr_point, double * point_list, int len, double thresh)
{
  int count = 0;
  int nlen = 0;
  int *use_pt;
  int i,j,k;
  double x,y,z;
  double cx,cy,cz,x1,y1,z1,x2,y2,z2,nd1,nd2,dst;
  double * normals;
  double *a;
  
  cx = curr_point[0];
  cy = curr_point[1];
  cz = curr_point[2];
  
  use_pt = malloc(len*sizeof(int));
  a = malloc(3*3*sizeof(double));
  
  for (i=0; i<len; i++)
  {
    x = point_list[3*i+0];
    y = point_list[3*i+1];
    z = point_list[3*i+2];
    
    if (norm(x,y,z) > 0.01 && distance(x,y,z,cx,cy,cz) > 0.01)
    {
      use_pt[i] = 1;
      count = count + 1;
    }
    else
    {
      use_pt[i] = 0;
    }
  }
  
  if (count > 1)
  {
  
    nlen = count*(count-1)/2;
  
    normals = malloc(3*nlen*sizeof(double));
  
    k = 0;
    for (i=0; i<len; i++)
    {
      if (use_pt[i] == 1)
      {
        for(j=i+1; j<len; j++)
        {
          if (use_pt[j] == 1)
          {
          
            x1 = point_list[3*i+0]-cx;
            y1 = point_list[3*i+1]-cy;
            z1 = point_list[3*i+2]-cz;
            nd1 = norm(x1,y1,z1);
            x1 = x1/nd1;
            y1 = y1/nd1;
            z1 = z1/nd1;
          
            x2 = point_list[3*j+0]-cx;
            y2 = point_list[3*j+1]-cy;
            z2 = point_list[3*j+2]-cz;
            nd2 = norm(x2,y2,z2);
            x2 = x2/nd2;
            y2 = y2/nd2;
            z2 = z2/nd2;        
          
            dst = distance(x1,y1,z1,x2,y2,z2);
          
            if (dst > 0.25 && dst < 1.75)
            {
            
              a[0*3+0] = x1;
              a[1*3+0] = y1;
              a[2*3+0] = z1;
          
              a[0*3+1] = x2;
              a[1*3+1] = y2;
              a[2*3+1] = z2;
          
              a[0*3+2] = 0;
              a[1*3+2] = 0;
              a[2*3+2] = 0;
          
              normal_3x3_with_point((normals+k*3), curr_point, a);
              if (norm(normals[3*k+0],normals[3*k+1],normals[3*k+2]) >0.9)
              {
                k = k+1;
              }
            }
          }
        }
      }
      
    }
    
    if (k > 0)
    {
      mode_normal_3x3(best_normal,normals,k,thresh);
    }
    else
    {
      best_normal[0] = 0;
      best_normal[1] = 0;
      best_normal[2] = 0;
    }
    
    free(normals);
  }
  else
  {
    best_normal[0] = 0;
    best_normal[1] = 0;
    best_normal[2] = 0;
  }
  
  free(a);
  free(use_pt);
  
}

int normal_list_for_point_3x3(double * normals, double * curr_point, double * point_list, int len)
{
  int count = 0;
  int *use_pt;
  int i,j,k;
  double x,y,z;
  double cx,cy,cz,x1,y1,z1,x2,y2,z2,nd1,nd2,dst;
  double *a;
  
  cx = curr_point[0];
  cy = curr_point[1];
  cz = curr_point[2];
  
  use_pt = malloc(len*sizeof(int));
  a = malloc(3*3*sizeof(double));
  
  for (i=0; i<len; i++)
  {
    x = point_list[3*i+0];
    y = point_list[3*i+1];
    z = point_list[3*i+2];
    
    if (norm(x,y,z) > 0.01 && distance(x,y,z,cx,cy,cz) > 0.01)
    {
      use_pt[i] = 1;
      count = count + 1;
    }
    else
    {
      use_pt[i] = 0;
    }
  }
  
  k = 0;
  if (count > 1)
  {
    for (i=0; i<len; i++)
    {
      if (use_pt[i] == 1)
        for(j=i+1; j<len; j++)
        {
          if (use_pt[j] == 1)
          {   
          
            x1 = point_list[3*i+0]-cx;
            y1 = point_list[3*i+1]-cy;
            z1 = point_list[3*i+2]-cz;
            nd1 = norm(x1,y1,z1);
            x1 = x1/nd1;
            y1 = y1/nd1;
            z1 = z1/nd1;
          
            x2 = point_list[3*j+0]-cx;
            y2 = point_list[3*j+1]-cy;
            z2 = point_list[3*j+2]-cz;
            nd2 = norm(x2,y2,z2);
            x2 = x2/nd2;
            y2 = y2/nd2;
            z2 = z2/nd2;        
          
            dst = distance(x1,y1,z1,x2,y2,z2);
          
            if (dst > 0.25 && dst < 1.75)
            {
            
              a[0*3+0] = x1;
              a[1*3+0] = y1;
              a[2*3+0] = z1;
          
              a[0*3+1] = x2;
              a[1*3+1] = y2;
              a[2*3+1] = z2;
          
              a[0*3+2] = 0;
              a[1*3+2] = 0;
              a[2*3+2] = 0;
          
              normal_3x3_with_point((normals+k*3), curr_point, a);
              if (norm(normals[3*k+0],normals[3*k+1],normals[3*k+2]) >0.9)
              {
                k=k+1;
              }
            }
          }
        }
      }
    }

  free(a);
  free(use_pt);
  return k;
  
}

int compute_normal(double* normal_map, double* point_map, int height, int width, int win, double thresh)
{
  int i,j,k1,k2,h,w,hh,ww,ret1,ret2;
  double * curr_point;
  double * point_list_1;
  double * point_list_2;
  double * best_normal;
  double x,y,z;
  int len = 4;
  
  curr_point = malloc(3*sizeof(double));
  best_normal = malloc(2*3* sizeof(double));
  point_list_1 = malloc(len*3*sizeof(double));
  point_list_2 = malloc(len*3*sizeof(double));
  
  for (i = 0; i< height; i++)
  {
    for (j= 0; j<width; j++)
    {
      
      curr_point[0] = point_map[0*height*width + i*width + j];
      curr_point[1] = point_map[1*height*width + i*width + j];
      curr_point[2] = point_map[2*height*width + i*width + j];
      
      if (norm(curr_point[0],curr_point[1],curr_point[2]) > 0.01)
      {
        
        k1 = 0;
        k2 = 0;
        
        for (h = -1; h <= 1; h++)
        {
          for (w = -1; w <= 1; w++)
          {
            
            hh = i + (h*win);
            ww = ((j + (w*win) + width) % width);
            
            if (hh >= 0 && hh < height && !(h == 0 && w == 0))
            {
              x = point_map[0*height*width + hh*width + ww];
              y = point_map[1*height*width + hh*width + ww];
              z = point_map[2*height*width + hh*width + ww];
              
              if (h != 0 && w != 0)
              {
                point_list_1[k1*3+0] = x;
                point_list_1[k1*3+1] = y;
                point_list_1[k1*3+2] = z;
                k1 = k1+1;
              }
              
              if(h == 0 || w == 0)
              {
                point_list_2[k2*3+0] = x;
                point_list_2[k2*3+1] = y;
                point_list_2[k2*3+2] = z;
                k2 = k2+1;
              } 
            }
            
          }
        }
        
        normal_for_point_3x3(best_normal, curr_point, point_list_1, k1, thresh);
        normal_for_point_3x3(best_normal+3, curr_point, point_list_2, k2, thresh);
        mode_normal_3x3(best_normal, best_normal, 2, thresh);
        
        normal_map[0*height*width + i*width + j] = best_normal[0];
        normal_map[1*height*width + i*width + j] = best_normal[1];
        normal_map[2*height*width + i*width + j] = best_normal[2];

      }
    }
  }
  
  free(best_normal);
  free(curr_point);
  free(point_list_1);
  free(point_list_2);
}

int compute_normal_variable_winsize(double* normal_map, double* point_map, int height, 
     int width, double win, int maxstep, double thresh)
{
  int i,j,k1,k2,h,w,hh,ww,ret1,ret2;
  double * curr_point;
  double * point_list_1;
  double * point_list_2;
  double * best_normal;
  int len = 4;
  int step;
  double x,y,z;
  double dst;
  
  curr_point = malloc(3*sizeof(double));
  best_normal = malloc(2*3* sizeof(double));
  point_list_1 = malloc(len*3*sizeof(double));
  point_list_2 = malloc(len*3*sizeof(double));
  
  for (i = 0; i< height; i++)
  {
    for (j= 0; j<width; j++)
    {
      
      curr_point[0] = point_map[0*height*width + i*width + j];
      curr_point[1] = point_map[1*height*width + i*width + j];
      curr_point[2] = point_map[2*height*width + i*width + j];
      
      if (norm(curr_point[0],curr_point[1],curr_point[2]) > 0.01)
      {
        k1 = 0;
        k2 = 0;
      
        for (h = -1; h <= 1; h++)
        {
          for (w = -1; w <= 1; w++)
          {

            step = 0;
            if (h!=0 || w !=0)
            {
              do 
              {
                step = step + 1;
                hh = i+(h*step);
                ww = (j+(w*step)+width) % width;
                dst = 0;
                if (hh >= 0 && hh < height)
                {
                  x = point_map[0*height*width + hh*width + ww];
                  y = point_map[1*height*width + hh*width + ww];
                  z = point_map[2*height*width + hh*width + ww];
                  dst = distance(x,y,z,curr_point[0],curr_point[1],curr_point[2]);
                }
              } while((step < maxstep) && (dst < win));
            }
            
            if (hh >= 0 && hh < height && !(h == 0 && w == 0))
            {
              if (h != 0 && w != 0)
              {
                point_list_1[k1*3+0] = x;
                point_list_1[k1*3+1] = y;
                point_list_1[k1*3+2] = z;
                k1 = k1+1;
              }
            
              if(h == 0 || w == 0)
              {
                point_list_2[k2*3+0] = x;
                point_list_2[k2*3+1] = y;
                point_list_2[k2*3+2] = z;
                k2 = k2+1;
              } 
            }
            
          }
        }
        
        normal_for_point_3x3(best_normal, curr_point, point_list_1, k1, thresh);
        normal_for_point_3x3(best_normal+3, curr_point, point_list_2, k2, thresh);
        mode_normal_3x3(best_normal, best_normal, 2, thresh);
        
        normal_map[0*height*width + i*width + j] = best_normal[0];
        normal_map[1*height*width + i*width + j] = best_normal[1];
        normal_map[2*height*width + i*width + j] = best_normal[2];
        
      }
      
    }
  }
  
  free(best_normal);
  free(curr_point);
  free(point_list_1);
  free(point_list_2);
}
*/


/*
int compute_normal_helper_pick(double *normals, double *cand_1, double *cand_2, 
                                                double *cand_3, double *cand_4, 
                                                int height, int width, double thresh) 
{

  double nx1,ny1,nz1, nx2,ny2,nz2, nx3,ny3,nz3, nx4,ny4,nz4;
  double dist12, dist13, dist14, dist23, dist24, dist34; 
  int indx,indy,indz;
  int h,w;
  
  for (h = 0; h < height; h++)
  {
    for (w = 0; w < width; w++)
    {
    
      double nrmx = 0,nrmy = 0,nrmz = 0;
      double avgx = 0,avgy = 0,avgz = 0;
      int divavg = 0;
      int divnrm = 0;
      
      indx = 0*height*width + h*width+w;
      indy = 1*height*width + h*width+w;
      indz = 2*height*width + h*width+w;
      
      nx1 = *(cand_1+indx);
      ny1 = *(cand_1+indy);
      nz1 = *(cand_1+indz);
      
      nx2 = *(cand_2+indx);
      ny2 = *(cand_2+indy);
      nz2 = *(cand_2+indz);
      
      nx3 = *(cand_3+indx);
      ny3 = *(cand_3+indy);
      nz3 = *(cand_3+indz);
      
      nx4 = *(cand_4+indx);
      ny4 = *(cand_4+indy);
      nz4 = *(cand_4+indz);
      
      if (norm(nx1,ny1,nz1) > 0.9)
      {
        avgx = avgx + nx1;
        avgy = avgy + ny1;
        avgz = avgz + nz1;
        divavg = divavg + 1;
        
        dist12 = distance(nx1,ny1,nz1,nx2,ny2,nz2);
        dist13 = distance(nx1,ny1,nz1,nx3,ny3,nz3);
        dist14 = distance(nx1,ny1,nz1,nx4,ny4,nz4);
        
        if ((dist12 < thresh) && (dist13 < thresh) && (dist14 < thresh))
        {
          divnrm = 4;
          nrmx = (nx1+nx2+nx3+nx4)/divnrm;
          nrmy = (ny1+ny2+ny3+ny4)/divnrm;
          nrmz = (nz1+nz2+nz3+nz4)/divnrm;
        } 
        else if ((dist12 < thresh) && (dist13 < thresh))
        {
          divnrm = 3;
          nrmx = (nx1+nx2+nx3)/divnrm;
          nrmy = (ny1+ny2+ny3)/divnrm;
          nrmz = (nz1+nz2+nz3)/divnrm;
        }
        else if ((dist12 < thresh) && (dist14 < thresh))
        {
          divnrm = 3;
          nrmx = (nx1+nx2+nx4)/divnrm;
          nrmy = (ny1+ny2+ny4)/divnrm;
          nrmz = (nz1+nz2+nz4)/divnrm;
        }
        else if ((dist13 < thresh) && (dist14 < thresh))
        {
          divnrm = 3;
          nrmx = (nx1+nx3+nx4)/divnrm;
          nrmy = (ny1+ny3+ny4)/divnrm;
          nrmz = (nz1+nz3+nz4)/divnrm;
        }
        else if ((dist12 < thresh))
        {
          divnrm = 2;
          nrmx = (nx1+nx2)/divnrm;
          nrmy = (ny1+ny2)/divnrm;
          nrmz = (nz1+nz2)/divnrm;
        }
        else if ((dist13 < thresh))
        {
          divnrm = 2;
          nrmx = (nx1+nx3)/divnrm;
          nrmy = (ny1+ny3)/divnrm;
          nrmz = (nz1+nz3)/divnrm;
        }
        else if ((dist14 < thresh))
        {
          divnrm = 2;
          nrmx = (nx1+nx4)/divnrm;
          nrmy = (ny1+ny4)/divnrm;
          nrmz = (nz1+nz4)/divnrm;
        }
      }
        
      if (norm(nx2,ny2,nz2) > 0.9)
      {
        avgx = avgx + nx2;
        avgy = avgy + ny2;
        avgz = avgz + nz2;
        divavg = divavg + 1;
        
        dist23 = distance(nx2,ny2,nz2,nx3,ny3,nz3);
        dist24 = distance(nx2,ny2,nz2,nx4,ny4,nz4);
        
        if ((dist23 < thresh) && (dist24 < thresh) && (divnrm < 3))
        {
          divnrm = 3;
          nrmx = (nx2+nx3+nx4)/divnrm;
          nrmy = (ny2+ny3+ny4)/divnrm;
          nrmz = (nz2+nz3+nz4)/divnrm;
        } 
        else if ((dist23 < thresh) && (divnrm < 2))
        {
          divnrm = 2;
          nrmx = (nx2+nx3)/divnrm;
          nrmy = (ny2+ny3)/divnrm;
          nrmz = (nz2+nz3)/divnrm;
        }
        else if ((dist24 < thresh) && (divnrm < 2))
        {
          divnrm = 2;
          nrmx = (nx2+nx4)/divnrm;
          nrmy = (ny2+ny4)/divnrm;
          nrmz = (nz2+nz4)/divnrm;
        }
      }
      
      if (norm(nx3,ny3,nz3) > 0.9)
      {
        avgx = avgx + nx3;
        avgy = avgy + ny3;
        avgz = avgz + nz3;
        divavg = divavg + 1;
        
        dist34 = distance(nx3,ny3,nz3,nx4,ny4,nz4);
        
        if ((dist34 < thresh) && (divnrm < 2))
        {
          divnrm = 2;
          nrmx = (nx3+nx4)/divnrm;
          nrmy = (ny3+ny4)/divnrm;
          nrmz = (nz3+nz4)/divnrm;
        }
      }
      
      if (norm(nx4,ny4,nz4) > 0.9)
      {
        avgx = avgx + nx4;
        avgy = avgy + ny4;
        avgz = avgz + nz4;
        divavg = divavg + 1;
      }
      
      if ((divnrm == 0) && (divavg > 0))
      {
        nrmx = avgx/divavg;
        nrmy = avgy/divavg;
        nrmz = avgz/divavg;
      }
      
      normals[indx] = nrmx;
      normals[indy] = nrmy;
      normals[indz] = nrmz;
      
    }
  }
  
  return 0;
}

int compute_normal_help_blend(double* output_normal, double* input_normal, 
                              double* xyz, int height, int width, 
                              int window, double dist_thresh, double norm_thresh, double point_thresh)
{

  int inh,inw;
  int minh,minw,maxh,maxw;
  int shh,shw;
  double nx,ny,nz,d,x,y,z;
  double shnx,shny,shnz,shd,shx,shy,shz;
  double dist,normdist;
  int indx,indy,indz;
  int shindx,shindy,shindz;
  
  for (inh = 0; inh < height; inh++)
  {
    for (inw = 0; inw < width; inw++)
    {
      double oupnx, oupny, oupnz;
      int div = 0;
      double factor = 0;
      
      indx = 0*height*width + inh*width+inw;
      indy = 1*height*width + inh*width+inw;
      indz = 2*height*width + inh*width+inw;
      
      nx = *(input_normal+indx);
      ny = *(input_normal+indy);
      nz = *(input_normal+indz);
      
      x = *(xyz+indx);
      y = *(xyz+indy);
      z = *(xyz+indz);
      
      d = find_d_of_plane(x,y,z,nx,ny,nz);
      
      oupnx = 0;
      oupny = 0;
      oupnz = 0;
      
      if (norm(nx,ny,nz) > 0.9 && norm(x,y,z) > 0.01)
      {
        minh = (int)fmax(0,inh-window);
        maxh = (int)fmin(height-1,inh+window);
        minw = inw-window+width;
        maxw = inw+window+width;
        
        for (shh = minh; shh <= maxh; shh++)
        {
          
          for (shw = minw; shw <= maxw; shw++)
          {
            
            shindx = 0*height*width + shh*width+(shw % width);
            shindy = 1*height*width + shh*width+(shw % width);
            shindz = 2*height*width + shh*width+(shw % width);
            
            shx = *(xyz+shindx);
            shy = *(xyz+shindy);
            shz = *(xyz+shindz);
            
            dist = distance_to_plane(shx,shy,shz,nx,ny,nz,d);
            
            if (dist < dist_thresh && distance(x,y,z,shx,shy,shz) < point_thresh && norm(shx,shy,shz) > 0.01)
            {
            
              shnx = *(input_normal+shindx);
              shny = *(input_normal+shindy);
              shnz = *(input_normal+shindz);
              
              shd = find_d_of_plane(shx,shy,shz,shnx,shny,shnz);
              
              normdist = distance(nx,ny,nz,shnx,shny,shnz);
              
              if (normdist < norm_thresh && norm(shnx,shny,shnz > 0.9))
              {
              
                oupnx = oupnx + shnx;
                oupny = oupny + shny;
                oupnz = oupnz + shnz;
              
                div = div + 1;
              }
            }
          }
        }
        
        if (div > 0)
        {
        
          oupnx = oupnx/div;
          oupny = oupny/div;
          oupnz = oupnz/div;
          
        }
      }
      
      output_normal[indx] = oupnx;
      output_normal[indy] = oupny;
      output_normal[indz] = oupnz;
      
    }
  }
  
  return 0;
}
*/