#include "utils.h"

//从文件中读取键值对
bool readConfigFile(const char* cfgfilepath, const string& key, string& value)
{
    fstream cfgFile;
    cfgFile.open(cfgfilepath);
    if (!cfgFile.is_open())
    {
        cout << "can not open cfg file!" << endl;
        return false;
    }
    char tmp[1000];
    while (!cfgFile.eof())//循环读取每一行
    {
        cfgFile.getline(tmp, 1000);
        string line(tmp);
        if (!line.compare(0, 2, "//")) continue;
        size_t pos = line.find('=');//找到每行的“=”号位置，之前是key之后是value
        if (pos == string::npos) return false;
        string tmpKey = line.substr(0, pos);//取=号之前
        if (key == tmpKey)
        {
            value = line.substr(pos + 1);//取=号之后
            return true;
        }
    }
    return false;
}

//字符串转向量
vec<3> getVec(string str) {
    vec3 vec = {};
    char* token = NULL;
    char* p = NULL;
    token = strtok_s(const_cast<char*>(str.c_str()), ",", &p);
    for (int i = 0; i < 3 && token != NULL; i++) {
        vec[i] = atof(token);
        token = strtok_s(NULL, ",", &p);
    }
    return vec;
}
