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
#define DIST_MIN 1

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
        dst_con = norm(ptx,pty,0)*100 + 1;
        
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
