#include "qutil.h"
#include <QFile>
#include <QString>
#include <QDebug>


std::string readResourceText(const QString& filename) {
  QFile file( filename );
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
      qWarning() << "Cannot open file " << filename;
      exit( EXIT_FAILURE );
  }
  
  char* data = file.readAll().data();
  file.close();
  
  return std::string(data);
}
