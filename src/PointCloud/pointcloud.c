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


int get_same_z_in_grid(double* xyz, int w, int h, double compz, int height, int width)
{

  int hh,i;
  int dir;
  double cur_x,cur_y,cur_z;
  double cdiff,bdifft=99999, bdiffl=-99999;
  double bht = -1,bhl = height;
  int offset = height*width;
  
  cur_x = xyz[0*offset+h*width+w];
  cur_y = xyz[1*offset+h*width+w];
  cur_z = xyz[2*offset+h*width+w];
  
  if (norm(cur_x,cur_y,cur_z) < 0.01)
  {
    //find top and bottom if best z is above h
    for(hh=h;hh>=0;hh--)
    {
        cur_x = xyz[0*offset+hh*width+w];
        cur_y = xyz[1*offset+hh*width+w];
        cur_z = xyz[2*offset+hh*width+w];
      
        if (norm(cur_x,cur_y,cur_z) > 0.01)
        {
          cdiff = cur_z-compz;
          
          if (cdiff == 0)
          {
            return hh;
          }
          
          if (cdiff < 0) // current is below comp
          {
            //check lower bound
            if (cdiff >= bdiffl)
            {
              bhl = hh;
              bdiffl = cdiff;
            }
          }
          
          else if (cdiff > 0) //current is above comp
          {
            //check top
            if (cdiff < bdifft)
            {
              bht = hh;
              bdifft = cdiff;
              break; //stop
            }
          }
         
        }
      }
    
    if (bhl-bht <= 1 && bhl-bht >= 0)
    {
        if (bhl == bht)
        {
          return bht;
        }
        if (fabs(bdiffl) <= fabs(bdifft))
        {
          return bhl;
        }
        else
        {
          return bht;
        }
      }
    
    //find top and bottom if best z is below h
    for(hh=h;hh<height;hh++)
    {
        cur_x = xyz[0*offset+hh*width+w];
        cur_y = xyz[1*offset+hh*width+w];
        cur_z = xyz[2*offset+hh*width+w];
      
        if (norm(cur_x,cur_y,cur_z) > 0.01)
        {
          cdiff = cur_z-compz;
          
          if (cdiff == 0)
          {
            return hh;
          }
          
          if (cdiff > 0) // current above desired
          {
            //check upper bound
            if (cdiff <= bdifft)
            {
              bht = hh;
              bdifft = cdiff;
            }
          }
          else if (cdiff < 0) // current below desired
          {
            //check lower bound
            if ((cdiff > bdiffl))
            {
              bhl = hh;
              bdiffl = cdiff;
              break; //stop
            }
          }
            
        }
      }
    
    if (bhl-bht <= 1 && bhl-bht >= 0)
    {
        if (bhl == bht)
        {
          return bht;
        }
        if (fabs(bdifft) <= fabs(bdiffl))
        {
          return bht;
        }
        else
        {
          return bhl;
        }
      }
    
    return -1;
    
  }
  else
  {
    if(cur_z < compz)
    {
      //need to go higher
      for(hh=h;hh>=0;hh--)
      {
        cur_x = xyz[0*offset+hh*width+w];
        cur_y = xyz[1*offset+hh*width+w];
        cur_z = xyz[2*offset+hh*width+w];
      
        if (norm(cur_x,cur_y,cur_z) > 0.01)
        {
          cdiff = cur_z-compz;
          
          if (cdiff == 0)
          {
            return hh;
          }
          
          if (cdiff < 0) // current is below comp
          {
            //check lower bound
            if (cdiff >= bdiffl)
            {
              bhl = hh;
              bdiffl = cdiff;
            }
          }
          
          else if (cdiff > 0) //current is above comp
          {
            //check top
            if (cdiff < bdifft)
            {
              bht = hh;
              bdifft = cdiff;
              break; //stop
            }
          }
         
        }
      }
    
      if (bhl-bht <= 1 && bhl-bht >= 0)
      {
        if (bhl == bht)
        {
          return bht;
        }
        if (fabs(bdiffl) <= fabs(bdifft))
        {
          return bhl;
        }
        else
        {
          return bht;
        }
      }
      else
      {
        return -1;
      }
    }
    else if(cur_z > compz) // current 
    {
      //need to go lower
      for(hh=h;hh<height;hh++)
      {
        cur_x = xyz[0*offset+hh*width+w];
        cur_y = xyz[1*offset+hh*width+w];
        cur_z = xyz[2*offset+hh*width+w];
      
        if (norm(cur_x,cur_y,cur_z) > 0.01)
        {
          cdiff = cur_z-compz;
          
          if (cdiff == 0)
          {
            return hh;
          }
          
          if (cdiff > 0) // current above desired
          {
            //check upper bound
            if (cdiff <= bdifft)
            {
              bht = hh;
              bdifft = cdiff;
            }
          }
          else if (cdiff < 0) // current below desired
          {
            //check lower bound
            if ((cdiff > bdiffl))
            {
              bhl = hh;
              bdiffl = cdiff;
              break; //stop
            }
          }
            
        }
      }
    
      if (bhl-bht <= 1 && bhl-bht >= 0)
      {
        if (bhl == bht)
        {
          return bht;
        }
        if (fabs(bdifft) <= fabs(bdiffl))
        {
          return bht;
        }
        else
        {
          return bhl;
        }
      }
      else
      {
        return -1;
      }
    }    
    else
    {
      return h;
    }  
}
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
          //sh = get_same_z_in_grid(xyz, sw, sh, zcur, height, width);
          
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


int downsample(double* downsampled_points, double* downsampled_rgb, 
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


int theta_map_var(double* theta_map, double* centered_point_map, 
                  int* step_left, int* step_right,
                  int height, int width)
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

int phi_map_var(double* phi_map, double* centered_point_map, 
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
  
  xyz_hw = malloc(3*sizeof(double));
  xyz_phw = malloc(3*sizeof(double));
  xyz_nhw = malloc(3*sizeof(double));
  
  
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

int remap_points(double* depth_sum, double* depth_num, double* depth_old, 
                 char* rmask_old, double* phi_old, double* theta_old,
                 int hght_old, int wdth_old, int hght_new, int wdth_new,
                 double phi_max, double phi_stp, double theta_max, double theta_stp)
{
  int i,j;
  int ci,ni,pi,cj,nj,pj;
  int coff,noff,poff;
  short r_cc;
  int ind_cc;
  double phi_cc, phi_lo, phi_hi, phi_tmp, phi_new;
  double the_cc, the_lo, the_hi, the_tmp, the_new;
  int r,c,ind_tmp, hlo,wlo,hhi,whi, h,w, ind_new;
  double depth_cc, wt_new, td;
  
  int* ind_on_row = malloc(2*sizeof(int));
  int* ind_on_col = malloc(2*sizeof(int));
  
  for(i = 0; i < hght_old; i++)
  {
  
    ci = i;
    ni = i+1;
    pi = i-1;
    
    if (pi == 0)
    {
      pi = i;
    }
    if (ni == hght_old)
    {
      ni = i;
    }
    
    coff = ci*wdth_old;
    noff = ni*wdth_old;
    poff = pi*wdth_old;
    
    for(j = 0; j< wdth_old; j++)
    {
    
      cj = j;
      nj = (j+1) % wdth_old;
      pj = (j-1 + wdth_old) % wdth_old;
      
      ind_cc = coff + cj;
      ind_on_col[0] = poff + cj;
      ind_on_col[1] = noff + cj;
      ind_on_row[0] = coff + pj;
      ind_on_row[1] = coff + nj;
      
      r_cc = rmask_old[ind_cc];
      
      if (r_cc == 0)
      {
        phi_cc = phi_old[ind_cc];
        the_cc = theta_old[ind_cc];
        depth_cc = depth_old[ind_cc];
        
        phi_lo = phi_cc;
        phi_hi = phi_cc;
        the_lo = the_cc;
        the_hi = the_cc;
        
        for (r = 0; r<2; r++)
        {
          ind_tmp = ind_on_col[r];
          if (rmask_old[ind_tmp] == 0)
          {
            phi_tmp = phi_old[ind_tmp];
            phi_lo = fmin(phi_lo,phi_tmp);
            phi_hi = fmax(phi_hi,phi_tmp);
          } 
        }
        for (c = 0; c<2; c++)
        {
          ind_tmp = ind_on_row[c];
          if (rmask_old[ind_tmp] == 0)
          {
            the_tmp = theta_old[ind_tmp];
            if (c == 0 && the_cc >= 0 && the_tmp < 0)
            {
              the_hi = PI;
            }
            else if (c == 1 && the_cc < 0 && the_tmp >= 0)
            {
              the_lo = 0;
            }
            else
            {
              the_lo = fmin(the_lo,the_tmp);
              the_hi = fmax(the_hi,the_tmp);
            }
          } 
        }
        
        phi_lo = MAX(phi_lo, phi_cc - 2*phi_stp);
        phi_hi = MIN(phi_hi, phi_cc + 2*phi_stp);
        the_lo = MAX(the_lo, the_cc - 2*theta_stp);
        the_hi = MIN(the_hi, the_cc + 2*theta_stp);
        
        hlo = MAX(0, floor((phi_max - phi_hi)/phi_stp));
        hhi = MIN(hght_new-1, ceil((phi_max - phi_lo)/phi_stp));
        wlo = MAX(0, floor((theta_max - the_hi)/theta_stp));
        whi = MIN(wdth_new-1, ceil((theta_max - the_lo)/theta_stp));
        
        for (h=hlo;h<=hhi;h++)
        {
          for(w=wlo;w<=whi;w++)
          {
            ind_new = h*wdth_old+w;
            phi_new = phi_max-h*phi_stp;
            the_new = theta_max-w*theta_stp;
            td = the_new-the_cc;
            if (td > PI)
            {
              td = td - 2*PI;
            }
            else if (td < -PI)
            {
              td = td + 2*PI;
            }
            
            wt_new = 1/(pow(phi_new-phi_cc,2)+pow(td,2)+1);
            
            depth_num[ind_new] = depth_num[ind_new]+wt_new;
            depth_sum[ind_new] = depth_sum[ind_new]+(wt_new*depth_cc);
          }
        }
        
      }      
    }
  }
  
  free(ind_on_row);
  free(ind_on_col);
  return 0;
}




