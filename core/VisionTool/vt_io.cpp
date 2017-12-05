#include "vt_io.h"

namespace vt
{
    bool loadBinary(const char *path, void **buffer)
    {
        *buffer = NULL;
        std::ifstream fs(path, std::ios::binary | std::ios::ate);
        if (fs.is_open())
        {
            int size = (int) fs.tellg();
            *buffer = new char[size];
            fs.seekg(0, std::ios::beg);
            fs.read((char*)*buffer, size);
            fs.close();
            return true;
        }
        return false;
    };

    bool loadBinary(const char *path, void *buffer)
    {
        std::ifstream fs(path, std::ios::binary | std::ios::ate);
        if (fs.is_open())
        {
            int size = (int)fs.tellg();
            fs.seekg(0, std::ios::beg);
            fs.read((char*)buffer, size);
            fs.close();
            return true;
        }
        return false;
    };

    void releaseBinary(char **buffer)
    {
        if (*buffer != NULL)
        {
            delete[] *buffer;
            *buffer = NULL;
        }
    }

    bool saveBinary(const char *name, const void *buf, int size)
    {
        std::ofstream fs(name, std::ios::binary | std::ios::trunc);
        if (fs.is_open())
        {
            fs.write((const char*)buf, size);
            return true;
        }
        return false;
    }

    std::vector<std::string> readList(std::string path)
    {
        std::vector<std::string> list;
#ifdef UNIX
        std::fstream fs(path.c_str());
#else
        std::fstream fs(path);
#endif
        char name[255];
        while (fs.getline(name, 255))
        {
            list.push_back(std::string(name));
        }
        return list;
    }
}
