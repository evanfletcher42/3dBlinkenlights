#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#include <opencv2/core.hpp>


typedef struct PointObservation
{
	typedef enum ObservationStatus
	{
		eUnprocessed = 0,  // Haven't looked at it
		eNoAssociation,    // Does not have a forward link
		eAssociated,       // Has a forward link
		eAmbiguousCode,    // Decoding LED index failed
		eLabeled,          // Has a valid LED index
		nStatuses
	} ObservationStatus_t;

	// Image location
	cv::Point2f p;

	// Average color in the blob
	double r;
	double g;
	double b;

	// Frame where this point was detected
	int nFrameIdx;

	// Detected LED index
	int nLedIdx;

	// Pointers to the previous and next observations of this LED
	struct PointObservation* pPrevFrameObs;
	struct PointObservation* pNextFrameObs;

	ObservationStatus_t status = ObservationStatus::eUnprocessed;

} PointObservation_t;

typedef enum LedColor_t
{
	eRed = 0,
	eGreen = 1,
	eBlue = 2,
	eAmbiguous = 3
} LedColor_t;

class PointTracker
{
public:
	PointTracker();
	~PointTracker();

	// Process a video into point observations.
	bool ExtractBlobsFromVideo( std::string videoPath );

	// Detect sync pulses in extracted RGB values of observations.
	bool DetectSyncPulses();

	// Track blobs frame-to-frame between specified frame IDs, based on position.
	bool AssociateBlobsOverTime( int nStartFrame, int nEndFrame, int nMaxMovementPixels );

	// Track LEDs between sync points.
	bool TrackLedsBetweenSyncPulses();

	// Assign LED IDs to observations between detected sync pulses.
	bool DecodeLEDs();

	// Convert a sequence of LED color observations into arrays of zero/one probabilities (and erasure probability)
	// Returns true if a codeword can be extracted from the specified range.
	// Returns false if the specified LED is not observed over the entire specified range.  
	bool ColorSequenceToCodes( const PointObservation_t &obs, int nSyncStartFrame, int nSyncEndFrame, double* pProbZero, double* pProbOne );

	// Decode a 3-bit word from a 4-bit Hadamard code.  
	// Returns true if successful, writing the decoded word to nDecoded.  
	// Returns false and sets nDecoded = -1 if the word cannot be decoded. 
	bool HadamardDecode(double* pProbZero, double* pProbOne, int& nDecoded);

	// Debug: Draw detections on a video.
	bool DrawTracksOnVideo(std::string videoPath);

	// Set of observed LEDs in each frame.  Index like: m_observations[ nFrameIdx ][ nObservationIdxInThisFrame ]
	std::vector< std::vector< PointObservation_t > > m_observations;

	static constexpr int k_nMaxLeds = 500;

private:

	// Frame IDs of centers of detected sync pulses.
	std::vector< int > m_syncCenters;

	static constexpr int k_nIntensityThresh = 21;
	static constexpr float k_flMinBlobAreaPixels = 15.0f;
	static constexpr float k_flMaxBlobAreaPixels = 1000.0f;
	static constexpr int k_nMaxBlobMotionFrameToFrame = 30;

	static constexpr int k_nMovingAverageSize = 5;
	static constexpr int k_nMaximaSeparation = 80;

	// Intensity delta between two frames required to interpret a bit as a one or a zero.
	// Prevents labeling of bright objects that aren't blinking LEDs (e.g. background lighting).
	static constexpr double k_flMinBitDist = 5.0;

	// (8, 4, 4)_2 augmented Hadamard code
	static constexpr int k_hadamardTable[16 * 8] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 1, 0, 1, 0, 1,
		0, 0, 1, 1, 0, 0, 1, 1,
		0, 1, 1, 0, 0, 1, 1, 0,
		0, 0, 0, 0, 1, 1, 1, 1,
		0, 1, 0, 1, 1, 0, 1, 0,
		0, 0, 1, 1, 1, 1, 0, 0,
		0, 1, 1, 0, 1, 0, 0, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 0, 1, 0, 1, 0, 1, 0,
		1, 1, 0, 0, 1, 1, 0, 0,
		1, 0, 0, 1, 1, 0, 0, 1,
		1, 1, 1, 1, 0, 0, 0, 0,
		1, 0, 1, 0, 0, 1, 0, 1,
		1, 1, 0, 0, 0, 0, 1, 1,
		1, 0, 0, 1, 0, 1, 1, 0
	};

};
