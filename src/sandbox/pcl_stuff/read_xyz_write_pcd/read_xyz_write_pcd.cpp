#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

int
main (int argc, char** argv)
{
  if (argc < 4)
  {
    cout << "oh no! \n";
    return -1;
  }
  
  char* option = argv[1];
  char* fname_in = argv[2];
  char* fname_out = argv[3];
  
  ifstream myfile_in;
  myfile_in.open(fname_in);
  
  if (!myfile_in.is_open())
  {
    cout << "Unable to open file " << fname_in << ".\n";
    return -1;
  }
  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  string line;
  
  int count = 0;
  int width = 0;
  int height = 0;
  
  while (myfile_in.good())
  {
    getline(myfile_in, line);
    istringstream iss(line);
    
    if (strcmp(option, "faro") == 0)
    {
      int cnt = 0;
      do
      {
        string sub;
        iss >> sub;
      
        if (cnt == 0)
        {
          int h = atoi(sub.c_str());
          if (h + 1 > height)
          {
            height = h+1;
          }
        }
      
        if (cnt == 1)
        {
          int w = atoi(sub.c_str());
          if (w + 1 > width)
          {
            width = w+1;
          }
        }
      
        cnt++;
      
      } while(iss && (cnt < 2));
      
    }
    
    count++;
  }
  
  myfile_in.close();
  
  cout << "width: " << width << "; height: " << height << "; count: " << count << endl;

  if (strcmp(option, "faro") == 0)
  {
    cloud.width = width*height;
    cloud.height = 1;
  }
  else if (strcmp(option, "cc") == 0)
  {
    cloud.width = count;
    cloud.height = 1;
  }
  
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  
  myfile_in.open(fname_in);
  
  if (!myfile_in.is_open())
  {
    cout << "Unable to open file " << fname_in << ".\n";
    return -1;
  }
  
  count = 0;
  
  while (myfile_in.good() && (count < cloud.points.size()))
  {
    getline(myfile_in, line);
    istringstream iss(line);
    int cnt = 0;
    do
    {
      string sub;
      iss >> sub;
      
      if (strcmp(option, "faro") == 0)
      {
        if (cnt == 2)
        {
          cloud.points[count].x = atof(sub.c_str());
        }
        if (cnt == 3)
        {
          cloud.points[count].y = atof(sub.c_str());
        }
        if (cnt == 4)
        {
          cloud.points[count].z = atof(sub.c_str());
        }
        if (cnt == 5)
        {
          cloud.points[count].r = atof(sub.c_str());
        }
        if (cnt == 6)
        {
          cloud.points[count].g = atof(sub.c_str());
        }
        if (cnt == 7)
        {
          cloud.points[count].b = atof(sub.c_str());
        }
      } else if (strcmp(option, "cc") == 0)
      {
      
        if (cnt == 0)
        {
          cloud.points[count].x = atof(sub.c_str());
        }
        if (cnt == 1)
        {
          cloud.points[count].y = atof(sub.c_str());
        }
        if (cnt == 2)
        {
          cloud.points[count].z = atof(sub.c_str());
        }
        if (cnt == 3)
        {
          cloud.points[count].r = atof(sub.c_str());
        }
        if (cnt == 4)
        {
          cloud.points[count].g = atof(sub.c_str());
        }
        if (cnt == 5)
        {
          cloud.points[count].b = atof(sub.c_str());
        }
      }
      
      cnt++;
      
    } while(iss);
    
    count++;
  }

  myfile_in.close();
  
  cout << "Saving " << cloud.points.size() << " data points to " << fname_out << endl;
  pcl::io::savePCDFileBinary(fname_out, cloud);
  cout << "Saved " << cloud.points.size() << " data points to " << fname_out << endl;
  
  return 0;
 
}
