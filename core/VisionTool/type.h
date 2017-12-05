#pragma once
#include "opencv2/core/core.hpp"
#include <fstream>

namespace vt
{
#ifdef VISIONTOOL_EXPORTS
#define VISIONTOOL_API __declspec(dllexport)
#else
#define VISIONTOOL_API
#endif
}
