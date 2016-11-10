#pragma once
#include <iostream>

#ifdef TRACKINGDLL_EXPORTS
#define  TRACKINGDLL_API __declspec(dllexport)
#else
#define TRACKINGDLL_API __declspec(dllimport)
#endif
