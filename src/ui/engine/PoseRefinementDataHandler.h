#ifndef POSE_REFINEMENT_DATA_HANDLER_H
#define POSE_REFINEMENT_DATA_HANDLER_H

#include <string>
#include <list>
#include <vector>
#include "Vectors.h"

using namespace std;

class Object;

struct VertexCalibrationData {
  Vector3 vertexPosition;
  Vector2 photoCoordinates;
  bool vertexPositionPicked;
  bool photoCoordinatesPicked;
  
  VertexCalibrationData(const Vector3& _vertexPosition, const Vector2& _photoCoordinates) {
    vertexPosition = _vertexPosition;
    photoCoordinates = _photoCoordinates;
    vertexPositionPicked = true;
    photoCoordinatesPicked = true;
  }
  
  VertexCalibrationData() {
    vertexPosition = Vector3({0.0, 0.0, 0.0});
    photoCoordinates = Vector2({0.0, 0.0});
    vertexPositionPicked = false;
    photoCoordinatesPicked = false;
  }
};

struct PoseRefinementData {
  string* photoFile;
  list<VertexCalibrationData*> calibrationData;
  
  PoseRefinementData(string* _photoFile) {
    photoFile = _photoFile;
    calibrationData.clear();
  }
  
  PoseRefinementData() {
    photoFile = NULL;
    calibrationData.clear();
  }
};

#define NUM_CALIBRATION_POINTS_REQUIRED 3

class PoseRefinementDataHandler {
public:
  PoseRefinementDataHandler(const string& _configFile);
  ~PoseRefinementDataHandler();
  
  Vector3 getStartPosition();
  string getModelPath();
  void setConfigFile(const string& _configFile);
  bool save(const string& _filename);
  
  void selectVertex(const Vector3& _vertex);
  void selectPhotoPoint(const Vector2& _photoPoint);
  void loadPhotoPlane();
  
private:
  void __deleteAllData();
  bool __configure();
  void __nextPhoto();
  
private:
  list<PoseRefinementData*> __data;
  PoseRefinementData* __workingData;
  
  string __configFile;
  string __modelFile;
  vector<string> __photoFiles;
  string __outputPath;
  
  Vector3 __startPosition;
  unsigned int workingPhotoIndex;
  Object* __photoPlane;
};

#endif