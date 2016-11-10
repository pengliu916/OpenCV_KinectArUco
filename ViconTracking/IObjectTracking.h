#pragma once
#include "ViconTrackingDLL.h"

namespace Tracking
{
	struct TRACKINGDLL_API Pos
	{
		double X;
		double Y;
		double Z;
		Pos(double x, double y, double z) :X(x), Y(y), Z(z){}
		Pos() :X(0), Y(0), Z(0){}
	};

	struct TRACKINGDLL_API Ori
	{
		double X;
		double Y;
		double Z;
		double W;
		Ori(double x, double y, double z, double w) :X(x), Y(y), Z(z), W(w){}
		Ori() :X(0), Y(0), Z(0), W(0){}
	};

	class TRACKINGDLL_API IObjectTracking
	{
	public:
		virtual ~IObjectTracking(){}
		virtual bool Initialize(const std::string& server) = 0;
		virtual bool ConnectServer() = 0;
		virtual void Disconnect() = 0;
		virtual bool IsConnected() = 0;
		virtual void Update() = 0;
		virtual void GetPosition(const std::string& subject, const std::string& segment, Pos& position) = 0;
		virtual void GetOrientation(const std::string& subject, const std::string& segment, Ori& orientation) = 0;
		virtual void GetRotationMat(const std::string& subject, const std::string& segment, double* pData) = 0;
		virtual std::string GetInfo() = 0;
	};

	class TRACKINGDLL_API ObjectTrackingFactory
	{
	public:
		static IObjectTracking* create();
	};
}