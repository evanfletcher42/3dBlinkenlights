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

	// Convert a sequence of LED color observations into 4-bit RGB codewords.
	// Returns true if a codeword can be extracted from the specified range.
	// Returns false if the specified LED is not observed over the entire specified range.  
	bool ColorSequenceToCodes( const PointObservation_t &obs, int nSyncStartFrame, int nSyncEndFrame, int &nCodeR, int &nCodeG, int &nCodeB );

	// Decode a 3-bit word from a 4-bit Hadamard code.  
	// Returns true if successful, writing the decoded word to nDecoded.  
	// Returns false and sets nDecoded = -1 if the word cannot be decoded. 
	bool HadamardDecode(const int code, int& nDecoded);

	// Returns the hamming distance between a and b.
	int HammingDist(const int a, const int b);

	// Debug: Draw detections on a video.
	bool DrawTracksOnVideo(std::string videoPath);

	// Set of observed LEDs in each frame.  Index like: m_observations[ nFrameIdx ][ nObservationIdxInThisFrame ]
	std::vector< std::vector< PointObservation_t > > m_observations;

private:

	// Frame IDs of centers of detected sync pulses.
	std::vector< int > m_syncCenters;

	static constexpr int k_nIntensityThresh = 7;
	static constexpr float k_flMinBlobAreaPixels = 15.0f;
	static constexpr float k_flMaxBlobAreaPixels = 1000.0f;
	static constexpr int k_nMaxBlobMotionFrameToFrame = 30;

	static constexpr int k_nMovingAverageSize = 10;
	static constexpr int k_nMaximaSeparation = 40;

	static constexpr int k_nMaxLedIdx = 500;

	// Intensity delta between two frames required to interpret a bit as a one or a zero.
	// Prevents labeling of bright objects that aren't blinking LEDs (e.g. background lighting).
	static constexpr double k_flMinBitDist = 5.0;

	// (8, 4, 2)_2 augmented Hadamard code
	static constexpr int k_hadamardTable[8] = {
		0b0000,
		0b0101,
		0b0010,
		0b0110,
		0b1111,
		0b1010,
		0b1100,
		0b1001
	};
};
