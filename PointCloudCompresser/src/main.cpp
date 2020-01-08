#include <string>
#include <iostream>
#include <vector>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>

#include "Octree.h"
#include "PointCloudIO.h"
#include "Encoder.h"
#include "Decoder.h"

using namespace CPC;

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
        << "Options:\n"
        << "\t-h,--help\t\tShow help message\n"
        << "\t-i,--input\tSpecify the input path, REQUIRED"
        << "\t-o,--output\tSpecify the output path, OPTIONAL will automatically detect the file extension and use the input file name"
        << std::endl;
}

int handleArgument(int argc, char* argv[], std::string& input, std::string& output, int& depth, int& forceDepth)
{
    if (argc < 2) {
        show_usage(argv[0]);
        return 1;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        }
        else if ((arg == "-i") || (arg == "--input")) {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                input = argv[++i]; // Increment 'i' so we don't get the argument as the next argv[i].
            }
            else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "--input option requires one argument." << std::endl;
                return 1;
            }
        }
        else if ((arg == "-o") || (arg == "--output")) {
            if (i + 1 < argc) {
                output = argv[++i];
            }
            else {
                std::cerr << "--output option requires one argument." << std::endl;
                return 1;
            }
        }
        else if ((arg == "-d") || (arg == "--depth")) {
            if (i + 1 < argc) {
                depth = std::stoi(argv[++i]);
            }
            else {
                std::cerr << "--depth option requires one argument." << std::endl;
                return 1;
            }
        }
        else if ((arg == "-f") || (arg == "--forceDepth")) {
            if (i + 1 < argc) {
                forceDepth = std::stoi(argv[++i]);
            }
            else {
                std::cerr << "--forceDepth option requires one argument." << std::endl;
                return 1;
            }
        }
    }
    return 0;
}

int main(int argc, char* argv[])
{
    std::string input, output;
    int depth = 16;
    int forceDepth = -1;

    int failed = -1;
    failed = handleArgument(argc, argv, input, output, depth, forceDepth);
    if (failed)
    {
        return failed;
    }

    // load the point cloud 
    boost::filesystem::path inputPath(input);

    // if input is ply, do encoding
    if (boost::iequals(inputPath.extension().c_str(), ".ply"))
    {
        if (output.empty())
            output = inputPath.parent_path().append(inputPath.stem().concat(".cpc").string()).string();

        // load ply
        std::cout << "Loading point cloud: " << inputPath.string() << std::endl;
        PointCloudIO io;
        auto pointCloud = io.loadPly(inputPath.string());

        float maxf = 0.12f;
        if (pointCloud.hasScalar)
        {
            for (int i = 0; i < pointCloud.positions.size(); ++i)
            {
                if (pointCloud.scalars[i] > maxf)
                {
                    pointCloud.scalars[i] = 0;
                }
            }
        }

        io.savePly(inputPath.string(), pointCloud);

        auto startTime = std::clock();
        // Generate Octree
        std::cout << "Generating Bottom-Up Octree..." << std::endl;
        Octree octree(depth, pointCloud);
        auto octreeTime = std::clock();

        // Encode
        std::cout << "Encoding Octree..." << std::endl;
        Encoder encoder;
        auto encodedData = encoder.encode(octree, (unsigned char)forceDepth);
        auto duration = std::clock() - startTime;
        std::cout << "Generation of Octree and Encoding Octree Timing " << octreeTime - startTime / (CLOCKS_PER_SEC / 1000) << " : " << std::clock() - octreeTime / (CLOCKS_PER_SEC / 1000) << std::endl;

        // write cpc
        std::cout << "Compressing and saving to " << output << std::endl;
        io.saveCpc(output, encodedData);
    }
    // if input is cpc, do decoding
    else if (boost::iequals(inputPath.extension().c_str(), ".cpc"))
    {
       if(output.empty())
          output = inputPath.parent_path().append(inputPath.stem().concat("_decoded.ply").string()).string();
           
        // load cpc
        std::cout << "Loading " << inputPath.string() << std::endl;
        PointCloudIO io;
        auto encodedData = io.loadCpc(inputPath.string());

        auto startTime = std::clock();
        // decoded into octree
        std::cout << "Decoding into Octree" << std::endl;
        Decoder decoder;
        auto octree = decoder.decode(encodedData);
        std::cout << "Decoding Timing " << std::clock() - startTime / (CLOCKS_PER_SEC / 1000) << std::endl;

        // write ply
        std::cout << "Writing decoded point cloud: " << output << std::endl;
        auto& pointCloud = octree.generatePointCloud();
        std::cout << "Num of Points: " << pointCloud.positions.size() << std::endl;
        io.savePly(output, pointCloud);
    }
    
    return 0;
}
