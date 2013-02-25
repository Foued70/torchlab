#include "PoseRefinementDataHandler.h"
#include "utils.h"
#include "Engine.h"
#include "Scene.h"
#include "Object.h"
#include "Texture.h"


PoseRefinementDataHandler::PoseRefinementDataHandler(const string& _configFile) :
  __configFile(_configFile),
  __startPosition(Vector3({0.0f, 0.0f, 0.0f})),
  __photoPlane(NULL),
  __workingData(NULL)
{
  __configure();
}

PoseRefinementDataHandler::~PoseRefinementDataHandler() {
  __deleteAllData();
}

void
PoseRefinementDataHandler::setConfigFile(const string& _configFile) {
  __configFile = _configFile;
}

void
PoseRefinementDataHandler::__deleteAllData() {
  while(!__data.empty()) {
    while(!__data.front()->calibrationData.empty()) {
      delete __data.front()->calibrationData.front();
      __data.front()->calibrationData.pop_front();
    }
    delete __data.front(); 
    __data.pop_front();
  } 
  if (__workingData) {
    while(!__workingData->calibrationData.empty()) delete __workingData->calibrationData.front(), __workingData->calibrationData.pop_front();
    delete __workingData;
  }
  
  __configFile = "";
  __modelFile = "";
  __photoFiles.clear();
  __outputPath = "";
  
  __startPosition = Vector3({0.0f, 0.0f, 0.0f});
}

Vector3 
PoseRefinementDataHandler::getStartPosition() {
  return __startPosition;
}

string 
PoseRefinementDataHandler::getModelPath() {
  return __modelFile;
}

bool
PoseRefinementDataHandler::__configure() {
  if (!__configFile.length()) {
    return false;
  }
  
  fstream configFileStream(__configFile.c_str(), ios::in);
  string buffer;
  
  __deleteAllData();
  
  while (!configFileStream.eof()) {
    //log(PARAM, "Reading configFile");
    getline (configFileStream, buffer);
    
		if (buffer[0] == '#')
			continue;	// comment
    
    if (buffer.substr(0, 6) == "Model=") {
			__modelFile = buffer.substr(6, buffer.length()-6);
      log(PARAM, "Read model file: %s", __modelFile.c_str());	
      continue;
    }
    
    if (buffer.substr(0, 6) == "Photo=") {
      __photoFiles.push_back( buffer.substr(6, buffer.length()-6) );
      log(PARAM, "Read photo file: %s", __photoFiles.back().c_str());	
      continue;
    }
    
    if (buffer.substr(0, 7) == "Output=") {
			__outputPath = buffer.substr(7, buffer.length()-7);
      log(PARAM, "Read output path: %s", __outputPath.c_str());	
      continue;
    }
    
    if (buffer.substr(0, 9) == "Position=") {
      istringstream lineData(buffer.substr(9, buffer.length()-9));
      lineData >> __startPosition.x >> __startPosition.y >> __startPosition.z;
      log(PARAM, "Read start position: %f %f %f", __startPosition.x, __startPosition.y, __startPosition.z);
      continue;
    }
  }
  
  if (  __modelFile.length() &&
        __photoFiles.size() &&
        __outputPath.length() ) 
  {
    workingPhotoIndex = 0;
    return true;
  }
  return false;
}

bool
PoseRefinementDataHandler::save(const string& _filename) {
  string saveFile = __outputPath + _filename;
  log(PARAM, "Saving pose alignment data to %s", saveFile.c_str());
  
  ofstream dataFile;
  dataFile.open (saveFile.c_str(), std::ios::out | std::ios::app);
  if( !dataFile.is_open() ) {
    dataFile.open(saveFile.c_str(), std::ios::in | std::ios::out | std::ios::trunc);
  }
  
  for(auto posIT = __data.begin(); posIT != __data.end(); posIT++) {
    dataFile << "Model=" << __modelFile.c_str() << "\n";
    dataFile << "Photo=" << (*(*posIT)->photoFile).c_str() << "\n";
    unsigned int vertexIndex = 1;
    for(auto vertIT = (*posIT)->calibrationData.begin(); vertIT != (*posIT)->calibrationData.end(); vertIT++) {
      dataFile << "Vertex" << vertexIndex << "=" << (*vertIT)->vertexPosition.x << " " << (*vertIT)->vertexPosition.y << " " << (*vertIT)->vertexPosition.z << "\n";
      dataFile << "Picture Coordinate" << vertexIndex << "=" << (*vertIT)->photoCoordinates.x << " " << (*vertIT)->photoCoordinates.y << "\n";
      vertexIndex++;
    }
    dataFile << "\n\n";
  }
  dataFile.close();
}

void 
PoseRefinementDataHandler::selectVertex(const Vector3& _vertex) {
  if (!__workingData) {
    __workingData = new PoseRefinementData();
    __workingData->calibrationData.push_back(new VertexCalibrationData());
    __workingData->photoFile = &__photoFiles[workingPhotoIndex];
  }
  __workingData->calibrationData.back()->vertexPosition = _vertex;
  __workingData->calibrationData.back()->vertexPositionPicked = true;
  
  if (  __workingData->calibrationData.back()->vertexPositionPicked &&  
        __workingData->calibrationData.back()->photoCoordinatesPicked )
  {
    if (__workingData->calibrationData.size() < NUM_CALIBRATION_POINTS_REQUIRED) {
      log(PARAM, "Calibration point pair added");
      __workingData->calibrationData.push_back(new VertexCalibrationData());
    }
    else {
      log(PARAM, "%d point pairs created. Calibration of current photo complete", NUM_CALIBRATION_POINTS_REQUIRED);
      __nextPhoto();
    }
  }
}

void 
PoseRefinementDataHandler::selectPhotoPoint(const Vector2& _photoPoint) {
  if (!__workingData) {
    __workingData = new PoseRefinementData();
    __workingData->calibrationData.push_back(new VertexCalibrationData());
    __workingData->photoFile = &__photoFiles[workingPhotoIndex];
  }
  __workingData->calibrationData.back()->photoCoordinates = _photoPoint;
  __workingData->calibrationData.back()->photoCoordinatesPicked = true;
  
  if (  __workingData->calibrationData.back()->vertexPositionPicked &&  
        __workingData->calibrationData.back()->photoCoordinatesPicked )
  {
    if (__workingData->calibrationData.size() < NUM_CALIBRATION_POINTS_REQUIRED) {
      log(PARAM, "Calibration point pair added");
      __workingData->calibrationData.push_back(new VertexCalibrationData());
    }
    else {
      log(PARAM, "%d point pairs created. Calibration of current photo complete", NUM_CALIBRATION_POINTS_REQUIRED);
      __nextPhoto();
    }
  }
}

void 
PoseRefinementDataHandler::loadPhotoPlane() {
  Scene* postScene = Engine::GetSingleton().getScene("PostScene");
  if (__photoPlane) {
    postScene->deleteObject("photoPlane");
  }
  std::vector<Texture*> texturesVector; 
  texturesVector.push_back(new Texture(__photoFiles[workingPhotoIndex].c_str(), MODE_INDEXED_MAP));
  texturesVector.push_back(FrameBuffer::GetSingleton().getTexture(PICKING_PASS));
      
  __photoPlane = postScene->createObject("photoPlane");
  if (!__photoPlane-> explicitLoad( "objects/planeNormalized.obj",
                                    0, //Dont flip the uvs
                                    "wireframeMat",
                                    Engine::GetSingleton().wireframeShader, 
                                    1.0f, 
                                    0.0f, 
                                    0.0f,
                                    &texturesVector[0],
                                    2) ) exit(1);
  texturesVector.clear();
}

void 
PoseRefinementDataHandler::__nextPhoto() {
  __data.push_back(__workingData);
  __workingData = NULL;
  workingPhotoIndex++;
  if (workingPhotoIndex >= __photoFiles.size()) {
    log(PARAM, "All photos calibrated! Saving Data.");
    save("PoseRefinmentCalibrationData.txt");
  }
  else {
    loadPhotoPlane();
  }
}