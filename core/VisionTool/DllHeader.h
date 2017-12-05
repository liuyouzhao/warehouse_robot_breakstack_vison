#pragma once 
#ifdef DLL_3D_TOOLS
#define DLL_EXPORT_IMPORT __declspec(dllexport)
#else
#define DLL_EXPORT_IMPORT __declspec(dllimport)
#endif