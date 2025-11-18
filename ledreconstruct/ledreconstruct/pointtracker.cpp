#include <algorithm>
#include <iostream>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "pointtracker.h"

PointTracker::PointTracker()
{
}

PointTracker::~PointTracker()
{
}

bool PointTracker::ExtractBlobsFromVideo(std::string videoPath)
{
    printf("Extracting blobs from video: %s\n", videoPath.c_str());
    cv::VideoCapture cap(videoPath);

    if (!cap.isOpened())
    {
        return false;
    }

    int nFrameIdx = 0;

    while (true)
    {
        cv::Mat img;
        cap >> img;
        if (img.empty())
        {
            break;
        }

        m_observations.emplace_back();

        // Extract bright blobs (LEDs, probably)

        cv::Mat imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        cv::Mat imgThresh;
        cv::threshold(imgGray, imgThresh, k_nIntensityThresh, 255, cv::THRESH_BINARY);

        // Remove hair
        cv::erode(imgThresh, imgThresh, cv::Mat(), cv::Point(-1, -1), 1);
        cv::dilate(imgThresh, imgThresh, cv::Mat(), cv::Point(-1, -1), 1);

        // Connect nearby blobs
        cv::dilate(imgThresh, imgThresh, cv::Mat(), cv::Point(-1, -1), 1);
        cv::erode(imgThresh, imgThresh, cv::Mat(), cv::Point(-1, -1), 1);

        std::vector< std::vector< cv::Point > > contours;
        cv::findContours(imgThresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Filter blobs and create point observations
        for (std::vector<cv::Point>& contour : contours)
        {
            double flArea = cv::contourArea(contour);
            if (flArea < k_flMinBlobAreaPixels || flArea > k_flMaxBlobAreaPixels)
            {
                continue;
            }

            // extract average color in blob.
            // For efficient averaging, crop out a bounding rectangle around the blob.  
            cv::Rect boundRect = cv::boundingRect(contour);

            cv::Mat imgCrop = img(boundRect);
            cv::Mat mask = cv::Mat::zeros(boundRect.size(), CV_8UC1);

            std::vector< cv::Point > contourOffset = contour;
            for (cv::Point& p : contourOffset)
            {
                p = p - boundRect.tl();
            }

            std::vector< std::vector< cv::Point > > singleContourArr(1, contourOffset);
            cv::drawContours(mask, singleContourArr, -1, 255, cv::FILLED);

            cv::Scalar blobColor = cv::mean(imgCrop, mask);

            // extract centroid of blob
            cv::Moments M = cv::moments(contour);
            double flBlobCx = M.m10 / M.m00;
            double flBlobCy = M.m01 / M.m00;

            // construct a new observation
            m_observations.back().emplace_back();
            PointObservation_t& obs = m_observations.back().back();
            obs.p = cv::Point2f((float)flBlobCx, (float)flBlobCy);
            obs.r = blobColor[2]; // OpenCV BGR -> r,g,b channels
            obs.g = blobColor[1];
            obs.b = blobColor[0];
            obs.nFrameIdx = nFrameIdx;
            obs.nLedIdx = -1;
            obs.pNextFrameObs = NULL;
            obs.pPrevFrameObs = NULL;
            obs.status = PointObservation::ObservationStatus::eUnprocessed;
        }

        //printf("Frame %04d: Detected %zu LED candidates\n", nFrameIdx, m_observations[nFrameIdx].size());

        if (nFrameIdx % 200 == 0)
        {
            printf("Processed %d frames...\n", nFrameIdx);
        }

        nFrameIdx++;
    }

    return true;
}

bool PointTracker::DetectSyncPulses()
{
    if (m_observations.size() == 0)
    {
        printf("DetectSyncPulses: Error: No observations\n");
        return false;
    }

    // RGB -> Mean intensity per frame
    std::vector<double> intensities(m_observations.size(), 0.0);
    for (int nFrameIdx = 0; nFrameIdx < m_observations.size(); nFrameIdx++)
    {
        for (int nObsIdx = 0; nObsIdx < m_observations[nFrameIdx].size(); nObsIdx++)
        {
            intensities[nFrameIdx] += m_observations[nFrameIdx][nObsIdx].r;
            intensities[nFrameIdx] += m_observations[nFrameIdx][nObsIdx].g;
            intensities[nFrameIdx] += m_observations[nFrameIdx][nObsIdx].b;
        }

        intensities[nFrameIdx] /= (m_observations.size() * 3);
    }

    printf("Intensities size: %zu\n", intensities.size());


    // 1D moving average
    cv::Mat1d intensitiesMat(1, intensities.size(), intensities.data());
    cv::Mat1d maIntensities;

    try
    {
        cv::blur(intensitiesMat, maIntensities, cv::Size(k_nMovingAverageSize, 1), cv::Point(-1, -1), cv::BORDER_REPLICATE);
    }
    catch (cv::Exception& e)
    {
        printf(e.what());
    }

    // Centers of sync pulses are local maxima in this 1D signal, with some minimum separation.  
    std::vector< std::pair< int, double > > localMaxima;
    for (int i = 1; i < intensities.size() - 1; i++)
    {
        if (maIntensities.at<double>(i) > maIntensities.at<double>(i-1) && 
            maIntensities.at<double>(i) > maIntensities.at<double>(i+1))
        {
            localMaxima.push_back(std::pair<int, double>(i, maIntensities.at<double>(i)));
        }
    }

    printf("Detected %zu local maxima\n", localMaxima.size());

    // Iteratively find highest peaks and remove shorter neighbors within the search window.
    m_syncCenters.clear();

    while (true)
    {
        int nLargestMaximaIdx = -1;
        double flLargestMaximaValue = -1.0;

        for (int i = 0; i < localMaxima.size(); i++)
        {
            if (localMaxima[i].second > flLargestMaximaValue)
            {
                nLargestMaximaIdx = localMaxima[i].first;
                flLargestMaximaValue = localMaxima[i].second;
            }
        }

        // Are we done?
        if (flLargestMaximaValue < 0)
            break;

        // Save out peak
        m_syncCenters.push_back(nLargestMaximaIdx);

        // Crush anything within k_nMaximaSeparation of this peak
        for (std::pair< int, double>& peak : localMaxima)
        {
            if (std::abs(peak.first - nLargestMaximaIdx) < k_nMaximaSeparation)
            {
                peak.first = -1;
                peak.second = -1.0;
            }
        }
    }

    // Sort peaks
    std::sort(m_syncCenters.begin(), m_syncCenters.end());

    printf("Detected %zu sync pulses\n", m_syncCenters.size());

    return m_syncCenters.size() > 0;
}

bool PointTracker::AssociateBlobsOverTime(int nStartFrame, int nEndFrame, int nMaxMovementPixels)
{
    if (nStartFrame > m_observations.size() || 
        nStartFrame > nEndFrame || 
        nEndFrame > m_observations.size() )
    {
        return false;
    }

    // Associate blobs between nStartFrame and nEndFrame.
    // Note that we only care about LEDs that can be tracked continuously from nStartFrame to nEndFrame, as these LEDs can be decoded.
    for (int nFrameIdx = nStartFrame+1; nFrameIdx < nEndFrame; nFrameIdx++)
    {
        std::vector< PointObservation_t >& prevFrame = m_observations[nFrameIdx - 1];
        std::vector< PointObservation_t >& currFrame = m_observations[nFrameIdx];

        std::vector< double > currFrameDistances(currFrame.size(), DBL_MAX);

        for (PointObservation_t &obsPrev : prevFrame)
        {
            PointObservation_t* pClosestCurrFrame = NULL;
            double flMinDist = DBL_MAX;
            int nClosestCurrFrameIdx = -1;

            for (int i = 0; i < currFrame.size(); i++)
            {
                double flDist = cv::norm(currFrame[i].p - obsPrev.p);

                if (flDist < flMinDist && flDist < currFrameDistances[i] && flDist < nMaxMovementPixels)
                {
                    pClosestCurrFrame = &(currFrame[i]);
                    flMinDist = flDist;
                    nClosestCurrFrameIdx = i;
                }
            }

            if (pClosestCurrFrame == NULL)
            {
                // No match for this observation in the current frame.  
                obsPrev.status = PointObservation::ObservationStatus::eNoAssociation;
                continue;
            }
             
            // Update pairing

            // If the point we found in the current frame was already associated with a point from the previous frame,
            // this means we have found a closer pairing, and should erase the previous frame's forward link.
            if (pClosestCurrFrame->pPrevFrameObs != NULL)
            {
                pClosestCurrFrame->pPrevFrameObs->pNextFrameObs = NULL;
                pClosestCurrFrame->pPrevFrameObs->status = PointObservation::ObservationStatus::eNoAssociation;
            }

            pClosestCurrFrame->pPrevFrameObs = &obsPrev;
            obsPrev.pNextFrameObs = pClosestCurrFrame;
            obsPrev.status = PointObservation::ObservationStatus::eAssociated;

            currFrameDistances[nClosestCurrFrameIdx] = flMinDist;
        }
    }

    return true;
}

bool PointTracker::TrackLedsBetweenSyncPulses()
{
    // At least two sync pulses are required to decode anything
    if (m_syncCenters.size() < 2)
        return false;

    for (int i = 0; i < m_syncCenters.size() - 1; i++)
    {
        int nSyncStart = m_syncCenters[i];
        int nSyncEnd = m_syncCenters[i + 1];

        AssociateBlobsOverTime(nSyncStart, nSyncEnd, k_nMaxBlobMotionFrameToFrame);
    }

    return true;
}

bool PointTracker::DecodeLEDs()
{
    // At least two sync pulses are required to decode anything
    if (m_syncCenters.size() < 2)
        return false;

    double probZero[32];
    double probOne[32];

    for (int i = 0; i < m_syncCenters.size() - 1; i++)
    {
        int nSyncStart = m_syncCenters[i];
        int nSyncEnd = m_syncCenters[i + 1];

        // Try to decode LEDs for every observation starting at nSyncStart
        for (PointObservation_t& obs : m_observations[nSyncStart])
        {
            //printf("Decoding sequence\n");

            if (!ColorSequenceToCodes(obs, nSyncStart, nSyncEnd, probZero, probOne))
                continue;

            // Decode in four parts (each 8-bit section to a 4-bit part of the LED code)
            int nLedIdx = 0;
            bool bDecodeSuccess = true;
            for (int j = 0; j < 4 && bDecodeSuccess; j++)
            {
                //printf("\tNibble idx %d\n", j);

                nLedIdx = nLedIdx << 4;
                int nDecodedNibble = 0;

                // Offset to matching block
                bDecodeSuccess &= HadamardDecode(probZero + j * 8, probOne + j * 8, nDecodedNibble);

                nLedIdx |= nDecodedNibble;
            }

            if (!bDecodeSuccess)
            {
                // Mark sequence as ambiguous
                PointObservation_t* pObs = &obs;
                while (pObs != NULL && pObs->nFrameIdx < nSyncEnd)
                {
                    pObs->status = PointObservation::ObservationStatus::eAmbiguousCode;
                    pObs = pObs->pNextFrameObs;
                }

                continue;
            }

            //printf("Decoded as: %d\n", nLedIdx);

            // Sanity check: Don't decode LEDs that are off the end of the string
            if (nLedIdx > k_nMaxLeds)
            {
                printf("Decode failure: %d > %d\n", nLedIdx, k_nMaxLeds);

                // Mark sequence as ambiguous
                PointObservation_t* pObs = &obs;
                while (pObs != NULL && pObs->nFrameIdx < nSyncEnd)
                {
                    pObs->status = PointObservation::ObservationStatus::eAmbiguousCode;
                    pObs = pObs->pNextFrameObs;
                }

                continue;
            }

            
            // Assign LED index in this observation's decoded range
            PointObservation_t* pObs = &obs;
            while (pObs != NULL && pObs->nFrameIdx < nSyncEnd)
            {
                pObs->nLedIdx = nLedIdx;
                pObs->status = PointObservation::ObservationStatus::eLabeled;
                pObs = pObs->pNextFrameObs;
            }
        }
    }

    return true;
}

bool PointTracker::ColorSequenceToCodes(const PointObservation_t& obs, int nSyncStartFrame, int nSyncEndFrame, double* pProbZero, double* pProbOne)
{
    // Input observation must be at the sync start frame
    if (obs.nFrameIdx != nSyncStartFrame)
    {
        //printf("Detection failure - Observation not on start frame idx\n");
        return false;
    }

    // LEDs blink in a looping 35 color sequence: 
    // W | [33 RGB ] | W * W [
    // nSyncStartFrame and nSyncEndFrame are on the edge between the two white blinks.
    // 
    // The first and last blinks (white) are the sync pulse and can be ignored.
    // The remaining 33 slots are colors, where the led is set to R, G, or B.
    // 
    // Bits are encoded as color transitions along the RGB cycle, where:
    //  - A positive cycle edge (RGB) is a 1
    //  - A negative cycle edge (BGR) is a 0 

    constexpr int k_nSlots = 35;
    constexpr int k_nColors = k_nSlots - 2;

    double flSlotWidth = (double)(nSyncEndFrame - nSyncStartFrame) / k_nSlots;

    // First extract slot colors as likelihoods (SVM-like)
    typedef struct ColorProb_t
    {
        double pRed;
        double pGrn;
        double pBlu;
    } ColorProb_t;

    ColorProb_t colors[k_nColors];

    //printf("\tColors: flSlotWidth %f\n", flSlotWidth);

    const PointObservation_t *pObs = &obs;

    //printf("\t\tR: %-5.01f G:%-5.01f B:%-5.01f\n", pObs->r, pObs->g, pObs->b);

    for (int i = 0; i < k_nColors; i++)
    {
        // Find frame ID closest to middle of color slot
        float flSlotCenter = nSyncStartFrame + flSlotWidth * (0.5 + i + 1);
        int nSlotFrame = (int)(flSlotCenter + 0.5);

        // Advance to frame
        while (pObs->nFrameIdx < nSlotFrame)
        {
            pObs = pObs->pNextFrameObs;
            if (pObs == NULL)
            {
                //printf("\t\tDetection failure: Track ends before next sync\n");
                return false;
            }

            //printf("\t\tR: %-5.01f G:%-5.01f B:%-5.01f\n", pObs->r, pObs->g, pObs->b);
        }

        // SVM-like color classifier
        double r = pObs->r;
        double g = pObs->g;
        double b = pObs->b;

        double flScoreR = 1.0 * r - 0.5 * g - 0.5 * b;
        double flScoreG = -0.5 * r + 1.0 * g - 0.5 * b;
        double flScoreB = -0.5 * r - 0.5 * g + 1.0 * b;
        double flMaxScore = MAX(flScoreR, MAX(flScoreG, flScoreB));

        // softmax
        double expR = std::exp(flScoreR - flMaxScore);
        double expG = std::exp(flScoreG - flMaxScore);
        double expB = std::exp(flScoreB - flMaxScore);
        double sumExp = expR + expG + expB;

        colors[i].pRed = expR / sumExp;
        colors[i].pGrn = expG / sumExp;
        colors[i].pBlu = expB / sumExp;

		//printf("\n\t\tpR: %0.02f pG: %0.02f pB: %0.02f\n\n", colors[i].pRed, colors[i].pGrn, colors[i].pBlu);
    }

    //while (pObs != NULL)
    //{
    //    pObs = pObs->pNextFrameObs;

    //    if(pObs != NULL)
    //        printf("\t\tR: %-5.01f G:%-5.01f B:%-5.01f\n", pObs->r, pObs->g, pObs->b);
    //}

    //printf("\tEdgeLikelihood:\n");

    // Decode into likelihoods for code zeros or ones
    // (Viterbi-like probabilistic decoder)
    for (int i = 0; i < k_nColors - 1; i++)
    {
        pProbOne[i] =
            colors[i].pRed * colors[i + 1].pGrn +
            colors[i].pGrn * colors[i + 1].pBlu +
            colors[i].pBlu * colors[i + 1].pRed;

        pProbZero[i] =
            colors[i].pRed * colors[i + 1].pBlu +
            colors[i].pBlu * colors[i + 1].pGrn +
            colors[i].pGrn * colors[i + 1].pRed;

        // p(erasure) = 1 - p(one) - p(zero)

   //     if (i % 8 == 0 && i > 0)
   //     {
   //         printf("\n");
   //     }

   //     if (pProbOne[i] > pProbZero[i])
   //     {
			//printf("\t\t1, p=%0.06f\n", pProbOne[i]);
   //     }
   //     else
   //     {
   //         printf("\t\t0, p=%0.06f\n", pProbZero[i]);
   //     }
    }

    return true;
}

bool PointTracker::HadamardDecode(double *pProbZero, double *pProbOne, int& nDecoded)
{
    // Probabilistic hadamard decoding: Compute likelihood of each codeword, choose most likely.

    double flBestMatchP = 0.0;
    double flSumP = 0.0;

    int nBestMatch = 0;

    for (int i = 0; i < 16; i++)
    {
        double flProb = 1.0;
        const int* pRow = k_hadamardTable + 8 * i;

        for (int j = 0; j < 8; j++)
        {
            double flProbErasure = 1.0 - pProbOne[j] - pProbZero[j];
            double flProbBitMatch = (pRow[j] == 1 ? pProbOne[j] : pProbZero[j]);

			flProb *= (flProbBitMatch + 0.5*flProbErasure);
        }

        //printf("\t\tCW: %02d p: %f\n", i, flProb);

        if (flProb > flBestMatchP)
        {
            flBestMatchP = flProb;
            nBestMatch = i;
        }

        flSumP += flProb;
    }

    flBestMatchP /= flSumP;

    const double k_flMinMatchP = 0.90;
    //printf("\tNormalized p(%d): %0.04f\n", nBestMatch, flBestMatchP);

    if (flBestMatchP < k_flMinMatchP)
    {
        // Too ambiguous
        //printf("\tDiscard (Ambiguous)");
        return false;
    }

    nDecoded = nBestMatch;
    return true;
}

bool PointTracker::DrawTracksOnVideo(std::string videoPath)
{
    cv::VideoCapture cap(videoPath);

    if (!cap.isOpened())
    {
        return false;
    }

    cv::namedWindow("Video", cv::WindowFlags::WINDOW_AUTOSIZE);

    int nFrameIdx = 0;
    while (true)
    {
        cv::Mat img;
        cap >> img;
        if (img.empty())
        {
            break;
        }

        int k_nRectRadiusPx = 15;

        cv::Scalar obsColorsByStatus[(int)(PointObservation::ObservationStatus::nStatuses)];

        obsColorsByStatus[PointObservation::eUnprocessed] = cv::Scalar(0, 0, 128);
        obsColorsByStatus[PointObservation::eNoAssociation] = cv::Scalar(0, 0, 255);
        obsColorsByStatus[PointObservation::eAssociated] = cv::Scalar(0, 128, 255);
        obsColorsByStatus[PointObservation::eAmbiguousCode] = cv::Scalar(0, 255, 255);
        obsColorsByStatus[PointObservation::eLabeled] = cv::Scalar(0, 255, 0);

        // Draw observations for this frame on top of the video
        for (PointObservation_t &obs : m_observations[nFrameIdx])
        {

            // Label the detection (if label known
            std::string label = "?";
            cv::Scalar color = obsColorsByStatus[obs.status];
            if (obs.nLedIdx != -1)
            {
                label = std::to_string(obs.nLedIdx);
            }

            // Draw a box around the detection
            cv::Point tl(obs.p.x - k_nRectRadiusPx, obs.p.y - k_nRectRadiusPx);
            cv::Point br(obs.p.x + k_nRectRadiusPx, obs.p.y + k_nRectRadiusPx);
            cv::rectangle(img, tl, br, color, 1, cv::LineTypes::LINE_AA);

            cv::putText(img, label, tl, cv::FONT_HERSHEY_PLAIN, 1.0, color, 1, cv::LineTypes::LINE_AA);

            // Draw the LED track since last sync
            PointObservation_t *pObs = &obs;
            while (pObs->pPrevFrameObs != NULL)
            {
                if (pObs->pPrevFrameObs == pObs)
                {
                    printf("Warning: Self associated observation??\n");
                    std::exit(1);
                }

                cv::line(img, pObs->p, pObs->pPrevFrameObs->p, color, 1, cv::LineTypes::LINE_AA);
                pObs = pObs->pPrevFrameObs;
            }
        }

        // Mark sync pulses
        if (std::find(m_syncCenters.begin(), m_syncCenters.end(), nFrameIdx) != m_syncCenters.end())
        {
            cv::putText(img, "SYNC", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 1, cv::LineTypes::LINE_AA);
        }

        cv::imshow("Video", img);

        if (cv::waitKey(16) >= 0)
            break;

        nFrameIdx++;
    }

    return true;
}
