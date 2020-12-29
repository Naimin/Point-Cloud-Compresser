#include <string>
#include <iostream>
#include <vector>
#include <random>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include "Octree.h"
#include "PointCloudIO.h"
#include "Encoder.h"
#include "Decoder.h"

using namespace CPC;

int pointcount = 0;
int memoryUseLevel[16];
int intersectiontimingLevel[16];
int classicTiming = 0;
int octreeTime = 0;
int encodeTime[16];
int compressSizeLevel[16];
int bottomup = 0;
int topdown = 0;
int zpfTime = 0;
int zpfSize = 0;
int merryTime = 0;
int merrySize = 0;


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

float get_random()
{
    static std::default_random_engine e;
    static std::uniform_real_distribution<> dis(0, 2); // range 0 - 1
    return (float)dis(e);
}

void testIntersection(EncodedData& data, PointCloud& pointCloud, int level)
{
    Decoder decoder;
    auto subNodePos = decoder.decodeNodeHeaders(data);
    Octree octree(data.maxDepth, data.sceneBoundingBox);

    // generate the random points
    /*srand(0);
    std::vector<Eigen::Vector3f> randPoints(5000000);

    float xSize = data.sceneBoundingBox.max.x() - data.sceneBoundingBox.min.x();
    float ySize = data.sceneBoundingBox.max.y() - data.sceneBoundingBox.min.y();
    float zSize = data.sceneBoundingBox.max.z() - data.sceneBoundingBox.min.z();

    for (auto& point : randPoints)
    {
        point.x() = data.sceneBoundingBox.min.x() + (get_random() * xSize);
        point.y() = data.sceneBoundingBox.min.y() + (get_random() * ySize);
        point.z() = data.sceneBoundingBox.min.z() + (get_random() * zSize);
    }*/

    int hitCounter = 0;
    int alreadyExist = 0;
    int subnodeNotFound = 0;
    int decodeNotFound = 0;
    int decodeFound = 0;


    auto startTime = std::clock();
    for(size_t i = 0; i < pointCloud.positions.size() * 0.05f; ++i)
    {
        auto& point = pointCloud.positions[i];
        intersectionState state;
        if (decoder.intersect(point, data, octree, subNodePos, state))
        {
            ++hitCounter;
        }
        /*
        switch (state)
        {
            case ALREADY_EXIST: ++alreadyExist; break;
            case SUBNODE_NOT_FOUND: ++subnodeNotFound; break;
            case DECODE_NOT_FOUND: ++decodeNotFound; break;
            case DECODE_FOUND: ++decodeFound; break;
        }*/
    }
//    std::cout << (std::clock() - startTime) / (CLOCKS_PER_SEC / 1000) << std::endl;
    intersectiontimingLevel[level] = (std::clock() - startTime) / CLOCKS_PER_SEC;

    // compute the memory usage of the octree
    size_t memoryUsed = 0;
    const size_t perNodeMem = sizeof(Index) + sizeof(Node);
    for (auto& level : octree.getLevels())
    {
        memoryUsed += level.size() * perNodeMem;
    }
    memoryUseLevel[level] = memoryUsed;

    //std::cout << "memory used:" << memoryUsed << std::endl;
    /*std::cout << "Intersection found: " << hitCounter << std::endl;
    std::cout << "ALREADY_EXIST: " << alreadyExist << std::endl;
    std::cout << "SUBNODE_NOT_FOUND: " << subnodeNotFound << std::endl;
    std::cout << "DECODE_NOT_FOUND: " << decodeNotFound << std::endl;
    std::cout << "DECODE_FOUND: " << decodeFound << std::endl;*/
}

void testZPF(size_t pointcount)
{
    std::stringstream command;
    command << "D:\\Master\\PointCloudCompresser\\thirdparty\\zpf\\simple.exe " << pointcount;
    std::cout << command.str() << std::endl;
    int error = system(command.str().c_str());
}

void testOctomap(std::string path)
{
    std::stringstream command;
    command << "D:\\Master\\PointCloudCompresser\\thirdparty\\octomap\\simple_example.exe " << path;
    std::cout << command.str() << std::endl;
    int error = system(command.str().c_str());
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
        pointcount = pointCloud.positions.size();

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
        octreeTime = (std::clock() - startTime) / CLOCKS_PER_SEC;
        bottomup = octree.bottomup;
        topdown = octree.topdown;

        std::vector<CPC::BoundingBox> boxes(16);
        std::vector<int> flag(pointCloud.positions.size());
        boxes[0] = octree.getBoundingBox();
        for (int i = 1; i < boxes.size(); ++i)
        {
            boxes[i].max = boxes[i - 1].max * get_random();
            boxes[i].min = boxes[i - 1].min * get_random();
        }
        
        auto bbTime = std::clock();
        for (int i = 0; i < pointCloud.positions.size() * 0.05; ++i)
        {
            auto point = pointCloud.positions[i];
            for (int j = 0; j < 17; ++j)
            {
                for (int k = 0; k < 18; ++k)
                {
                    if (boxes[j].isInside(point))
                    {
                        flag[i] += rand();
                    }
                    else
                    {
                        flag[i] += rand() + 1;
                    }
                }
            }
        }
        //std::cout << "Bounding box Timing " << (std::clock() - bbTime) / (CLOCKS_PER_SEC / 1000) << std::endl;
        classicTiming = (std::clock() - bbTime) / CLOCKS_PER_SEC;
        
        for (int i = 0; i < 16; ++i)
        {
            //int i = 12;
            // Encode
            //std::cout << "Encoding Octree..." << std::endl;
            auto encodeStart = std::clock();
            Encoder encoder;
            auto encodedData = encoder.encode(octree, i);
            auto duration = std::clock() - startTime;
            encodeTime[i] = (std::clock() - encodeStart) / CLOCKS_PER_SEC;
            //std::cout << "Generation of Octree and Encoding Octree Timing " << octreeTime - startTime / (CLOCKS_PER_SEC / 1000) << " : " << std::clock() - octreeTime / (CLOCKS_PER_SEC / 1000) << std::endl;
            //std::cout << "Level " << i << std::endl;
            // Run the intersection test
            testIntersection(encodedData, pointCloud, i);

            auto out_index = inputPath.parent_path().append(inputPath.stem().concat(std::to_string(i)).concat(".cpc").string()).string();
            io.saveCpc(out_index, encodedData);

            compressSizeLevel[i] = boost::filesystem::file_size(out_index);
        }
        // write cpc
        std::cout << "Compressing and saving to " << output << std::endl;
        //io.saveCpc(output, encodedData);
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
    
    testZPF(pointcount);
    testOctomap(input);
    std::cout << input << std::endl;
    std::cout << pointcount << std::endl;
    std::cout << boost::filesystem::file_size(input) << std::endl;
    std::cout << classicTiming << std::endl;
    std::cout << octreeTime << std::endl;
    std::cout <<bottomup << std::endl;
    std::cout << topdown << std::endl;
    for(int i = 0; i < 16; ++i)
        std::cout << memoryUseLevel[i] << std::endl;
    for (int i = 0; i < 16; ++i)
        std::cout << intersectiontimingLevel[i] << std::endl;
    for (int i = 0; i < 16; ++i)
        std::cout << compressSizeLevel[i] / 1024 << std::endl;
    for (int i = 0; i < 16; ++i)
        std::cout << encodeTime[i] << std::endl;

    return 0;
}
