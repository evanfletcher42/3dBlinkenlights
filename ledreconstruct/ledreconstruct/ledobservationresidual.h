#pragma once

#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "pointtracker.h"


// ceres-solver residual for the bundle adjustment problem
struct LedObservationResidual {

	LedObservationResidual(double flObservedX, double flObservedY)
		: m_flObservedX(flObservedX)
		, m_flObservedY(flObservedY)
	{
	}

	static ceres::CostFunction* Create(const PointObservation_t &obs )
	{
		return (new ceres::AutoDiffCostFunction<
			LedObservationResidual, 2, 6, 9, 3>(
				new LedObservationResidual(obs.p.x, obs.p.y)));
	}

	// OpenCV-like pinhole camera distortion model
	enum CameraIntrinsicsTraits
	{
		eFx = 0,
		eFy,
		eCx,
		eCy,
		eK1,
		eK2,
		eP1,
		eP2,
		eK3,
		nIntrinsicsTraits
	};

	template < typename T > inline bool ProjectPointToCamera(const T* const pInCamera, const T* const cameraIntrinsics, T* pInImage) const
	{
		// Points should not be behind the camera
		if (pInCamera[2] <= T(0))
			return false;

		// Normalized camera coordinates
		T xp = pInCamera[0] / pInCamera[2];
		T yp = pInCamera[1] / pInCamera[2];

		T r2 = xp * xp + yp * yp;
		T r4 = r2 * r2;
		T r6 = r4 * r2;

		// Radial distortion
		T gamma = T(1.0) +
			cameraIntrinsics[eK1] * r2 +
			cameraIntrinsics[eK2] * r4 +
			cameraIntrinsics[eK3] * r6;

		// Tangential distortion
		T xpp = xp * gamma +
			T(2) * cameraIntrinsics[eP1] * xp * yp +
			cameraIntrinsics[eP2] * (r2 + T(2) * xp * xp);

		T ypp = yp * gamma +
			cameraIntrinsics[eP1] * (r2 + T(2) * yp * yp) +
			T(2) * cameraIntrinsics[eP2] * xp * yp;

		// Focal length & principal point
		pInImage[0] = cameraIntrinsics[eFx] * xpp + cameraIntrinsics[eCx];
		pInImage[1] = cameraIntrinsics[eFy] * ypp + cameraIntrinsics[eCy];

		return true;
	}

	template < typename T > bool operator()(const T* const cameraFromWorld, const T* const cameraIntrinsics, const T* const pInWorld, T* residuals) const
	{
		T pInCamera[3];
		ceres::AngleAxisRotatePoint(cameraFromWorld, pInWorld, pInCamera);
		
		pInCamera[0] += cameraFromWorld[3];
		pInCamera[1] += cameraFromWorld[4];
		pInCamera[2] += cameraFromWorld[5];

		T pInImage[2];
		if (!ProjectPointToCamera(pInCamera, cameraIntrinsics, pInImage))
		{
			return false;
		}

		residuals[0] = pInImage[0] - T(m_flObservedX);
		residuals[1] = pInImage[1] - T(m_flObservedY);

		return true;
	}

	double m_flObservedX;
	double m_flObservedY;
};

// L2 norm that preserves sane gradients when inputs are near zero.
template <typename T> T JetSafeL2Norm(const T* const v, int ndim)
{
	T result = T(0.0);

	for (int i = 0; i < ndim; i++)
	{
		result += v[i] * v[i];
	}

	if (result > std::numeric_limits<double>::epsilon())
	{
		result = ceres::sqrt(result);
	}

	return result;
}

// Residual for aligning the reconstructed tree along the z axis.  
struct AlignmentResidual
{
	AlignmentResidual(const double* const pPInWorld) :
		m_pPInWorld(pPInWorld)
	{
	}

	static ceres::CostFunction* Create(const double* const pPInWorld)
	{
		return (new ceres::AutoDiffCostFunction< 
			AlignmentResidual, 2, 6>( 
				new AlignmentResidual(pPInWorld)));
	}

	template <typename T> bool operator()(const T* const alignedFromWorld, T* residuals) const
	{
		T pInWorld[3];
		for (int i = 0; i < 3; i++)
		{
			pInWorld[i] = T(m_pPInWorld[i]);
		}

		T pAligned[3];
		ceres::AngleAxisRotatePoint(alignedFromWorld, pInWorld, pAligned);

		pAligned[0] += alignedFromWorld[3];
		pAligned[1] += alignedFromWorld[4];
		pAligned[2] += alignedFromWorld[5];

		// First residual: Distance from origin
		residuals[0] = JetSafeL2Norm(pAligned, 3);

		// Second residual: Distance from Z axis
		residuals[1] = JetSafeL2Norm(pAligned, 2);

		return true;
	}

private:
	const double* const m_pPInWorld;
};