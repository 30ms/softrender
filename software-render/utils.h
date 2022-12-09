#include <fstream>
#include "geometry.h"
using namespace std;

bool readConfigFile(const char* cfgfilepath, const string& key, string& value);

vec<3> getVec(string str);