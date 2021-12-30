#include <iostream>

#include <math.h>
#include <string>

#include <ceres/ceres.h>
#include <cxxopts.hpp>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include "bundleadjustment.h"
#include "pointtracker.h"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    // Command line options
    cxxopts::Options options("ledreconstruct", "Reconstruct 3D locations of LEDs on an addressible light string.");
    options.add_options()
        ("v,video", "Path to input video", cxxopts::value<std::string>())
        ("c,calibration", "Path to camera calibration JSON file", cxxopts::value<std::string>())
        ("s,solution", "Output unaligned solution JSON path (optional)", cxxopts::value<std::string>())
        ("o,output", "Output aligned points JSON path", cxxopts::value<std::string>());

    // Path to the video to process.
    std::string videoPath;
    
    // Path to the camera calibration JSON file.
    std::string calPath;

    // (Optional) path for output solver solution, with camera poses and points in an unaligned csys.
    std::string outSolutionPath;
    bool bOutputSolution = false;

    // Path for output reconstructed points only, in an aligned csys.
    std::string outJsonPath;

    try
    {
        cxxopts::ParseResult result = options.parse(argc, argv);

        if (result.count("video"))
        {
            videoPath = result["video"].as<std::string>();
        }
        else
        {
            throw cxxopts::option_required_exception("video");
        }

        if (result.count("calibration"))
        {
            calPath = result["calibration"].as<std::string>();
        }
        else
        {
            throw cxxopts::option_required_exception("calibration");
        }

        if (result.count("solution"))
        {
            bOutputSolution = true;
            outSolutionPath = result["output"].as<std::string>();
        }

        if (result.count("output"))
        {
            outJsonPath = result["output"].as<std::string>();
        }
        else
        {
            throw cxxopts::option_required_exception("output");
        }

    }
    catch (cxxopts::OptionException& e)
    {
        printf(e.what());
        printf("\n");
        printf(options.help().c_str());

        return 1;
    }

    // Process
    PointTracker tracker;

    if (!tracker.ExtractBlobsFromVideo(videoPath))
    {
        printf("Failed to extract blobs from video\n");
        return 1;
    }

    tracker.DetectSyncPulses();

    tracker.TrackLedsBetweenSyncPulses();

    tracker.DecodeLEDs();

    // Debug
    tracker.DrawTracksOnVideo(videoPath);

    BundleAdjuster bundleAdjuster(&tracker.m_observations);

    if (!bundleAdjuster.LoadCameraCalibrationFromJson(calPath))
    {
        printf("Error: Failed to load calibration json: %s\n", calPath.c_str());
        return 1;
    };

    bundleAdjuster.Reconstruct();
    bundleAdjuster.FindAlignmentTransform();

    if (bOutputSolution)
    {
        printf("Writing full solution to: %s\n", outSolutionPath.c_str());
        bundleAdjuster.WriteSolutionToJson(outJsonPath);
    }

    printf("Writing aligned point cloud to: %s\n", outJsonPath.c_str());
    bundleAdjuster.WriteAlignedReconstructionToJson(outJsonPath);

    bundleAdjuster.DrawReconstructionOnVideo(videoPath);

    return 0;
}
