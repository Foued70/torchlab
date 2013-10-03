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
#define DIST_MAX 9999

double distance(double nx1, double ny1, double nz1,
                double nx2, double ny2, double nz2)
{
  return sqrt(pow(nx1-nx2,2) + pow(ny1-ny2,2) + pow(nz1-nz2,2));
}

double norm(double nx, double ny, double nz)
{
  return distance(nx,ny,nz,0,0,0);
}

double find_d_of_plane(double x, double y, double z, double nx, double ny, double nz)
{
  return (0 - (x*nx + y*ny + z*nz));
}

double distance_to_plane(double x, double y, double z, double nx, double ny, double nz, double d)
{
  return (fabs(x*nx + y*ny + z*nz + d) / sqrt(pow(nx,2)+pow(ny,2)+pow(nz,2)));
}

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

int compute_normal_help_blend(double* output_normal, double * output_d, 
                              double * output_xyz, double* input_normal, 
                              double* xyz, int height, int width, 
                              int window, double dist_thresh, double norm_thresh)
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
      double oupnx, oupny, oupnz, oupd, oupx, oupy,oupz;
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
      
      oupx = 0;
      oupy = 0;
      oupz = 0;
      
      oupd = 0;
      
      if (norm(nx,ny,nz) > 0.9)
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
            
            if (dist < dist_thresh)
            {
            
              shnx = *(input_normal+shindx);
              shny = *(input_normal+shindy);
              shnz = *(input_normal+shindz);
              
              shd = find_d_of_plane(shx,shy,shz,shnx,shny,shnz);
              
              normdist = distance(nx,ny,nz,shnx,shny,shnz);
              
              if (normdist < norm_thresh)
              {
              
                oupnx = oupnx + shnx;
                oupny = oupny + shny;
                oupnz = oupnz + shnz;
                
                oupd = oupd + shd;
              
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
          
          oupd = oupd/div;
          
          factor = oupnx * x + oupny * y + oupnz * z;
          if (factor != 0)
          {
            factor = -oupd / (factor);
          }
          else
          {
            factor = 1;
          }
          
          oupx = x * factor;
          oupy = y * factor;
          oupz = z * factor;
          
          if (distance(x,y,z,oupx,oupy,oupz) > dist_thresh)
          {
            oupx = x;
            oupy = y;
            oupz = z;
          }
        }
      }
      
      output_normal[indx] = oupnx;
      output_normal[indy] = oupny;
      output_normal[indz] = oupnz;
      
      output_d[indx] = oupd;
      
      output_xyz[indx] = oupx;
      output_xyz[indy] = oupy;
      output_xyz[indz] = oupz;
      
    }
  }
  
  return 0;
}

int downsample_with_panorama(double* downsampled_points, double* downsampled_rgb, 
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
        
        if ((norm(ptx,pty,ptz) > 0) && (norm(ptx,pty,ptz) < DIST_MAX))
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

int downsample_without_panorama(double* downsampled_points, double* downsampled_rgb, 
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
