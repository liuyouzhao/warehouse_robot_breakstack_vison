#pragma once
#include "type.h"

namespace vt
{
    VISIONTOOL_API bool loadBinary(const char *path, void **buffer);
    VISIONTOOL_API bool loadBinary(const char *path, void *buffer);

    VISIONTOOL_API void releaseBinary(char **buffer);

    VISIONTOOL_API bool saveBinary(const char *name, const void *buf, int size);

    VISIONTOOL_API std::vector<std::string> readList(std::string path);

}