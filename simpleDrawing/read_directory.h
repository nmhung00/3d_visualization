#ifndef READ_DIRECTORY_H
#define READ_DIRECTORY_H

#include <algorithm>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <vector>
#include <string>

using namespace std;

vector<string> read_directory(string directory);

#endif
