#include "PointCloudIO.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <boost/filesystem.hpp>
#include <Boost/filesystem/path.hpp>
#include <sstream>
#include "Huffman.h"

#define TINYPLY_IMPLEMENTATION

using namespace CPC;
using namespace tinyply;

class manual_timer
{
    std::chrono::high_resolution_clock::time_point t0;
    double timestamp{ 0.f };
public:
    void start() { t0 = std::chrono::high_resolution_clock::now(); }
    void stop() { timestamp = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - t0).count() * 1000; }
    const double & get() { return timestamp; }
};

CPC::PointCloudIO::PointCloudIO()
{
}

CPC::PointCloudIO::~PointCloudIO()
{
}

PointCloud CPC::PointCloudIO::loadPly(const std::string & path)
{
    try
    {
        std::ifstream ss(path, std::ios::binary);
        if (ss.fail()) throw std::runtime_error("failed to open " + path);

        PlyFile file;
        file.parse_header(ss);

        std::cout << "........................................................................\n";
        for (auto c : file.get_comments()) std::cout << "Comment: " << c << std::endl;
        for (auto e : file.get_elements())
        {
            std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
            for (auto p : e.properties) std::cout << "\tproperty - " << p.name << " (" << tinyply::PropertyTable[p.propertyType].str << ")" << std::endl;
        }
        std::cout << "........................................................................\n";

        // Tinyply treats parsed data as untyped byte buffers. See below for examples.
        std::shared_ptr<PlyData> vertices, normals, colors;

        // The header information can be used to programmatically extract properties on elements
        // known to exist in the header prior to reading the data. For brevity of this sample, properties 
        // like vertex position are hard-coded: 
        try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { normals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }, (unsigned int)vertices->count); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        try { colors = file.request_properties_from_element("vertex", { "red", "green", "blue" }, (unsigned int)vertices->count); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        // Providing a list size hint (the last argument) is a 2x performance improvement. If you have 
        // arbitrary ply files, it is best to leave this 0. 
        //try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
        //catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        manual_timer read_timer;

        read_timer.start();
        file.read(ss);
        read_timer.stop();

        std::cout << "Reading took " << read_timer.get() / 1000.f << " seconds." << std::endl;
        if (vertices) std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
        if (normals) std::cout << "\tRead " << normals->count << " total vertex normals " << std::endl;
        if (colors) std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;

        PointCloud ptCloud(normals.operator bool(), colors.operator bool());
        ptCloud.resize(vertices->count);

        // type casting to your own native types - Option A
        {
            const size_t numBytes = vertices->buffer.size_bytes();
            std::memcpy(ptCloud.positions.data(), vertices->buffer.get(), numBytes);
            /*
            if (normals)
            {
                const size_t numBytes = normals->buffer.size_bytes();
                std::memcpy(ptCloud.normals.data(), normals->buffer.get(), numBytes);
            }
            
            if (colors)
            {
                const size_t numBytes = colors->buffer.size_bytes();
                std::memcpy(ptCloud.colors.data(), colors->buffer.get(), numBytes);
            }*/
        }

        return ptCloud;
    }
    catch (const std::exception & e)
    {
        std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    }

    return PointCloud();
}

bool CPC::PointCloudIO::savePly(const std::string & path, PointCloud & pointCloud)
{
    std::filebuf fb_binary;
    fb_binary.open(path, std::ios::out | std::ios::binary);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + path);

    /*std::filebuf fb_ascii;
    fb_ascii.open(path + "-ascii.ply", std::ios::out);
    std::ostream outstream_ascii(&fb_ascii);
    if (outstream_ascii.fail()) throw std::runtime_error("failed to open " + path);*/

    PlyFile plyFile;

    plyFile.add_properties_to_element("vertex", { "x", "y", "z" },
        Type::FLOAT32, pointCloud.positions.size(), reinterpret_cast<uint8_t*>(pointCloud.positions.data()), Type::INVALID, 0);

    //cube_file.add_properties_to_element("vertex", { "nx", "ny", "nz" },
    //    Type::FLOAT32, cube.normals.size(), reinterpret_cast<uint8_t*>(cube.normals.data()), Type::INVALID, 0);

    //cube_file.add_properties_to_element("vertex", { "u", "v" },
        //Type::FLOAT32, cube.texcoords.size(), reinterpret_cast<uint8_t*>(cube.texcoords.data()), Type::INVALID, 0);

    //cube_file.add_properties_to_element("face", { "vertex_indices" },
      //  Type::UINT32, cube.triangles.size(), reinterpret_cast<uint8_t*>(cube.triangles.data()), Type::UINT8, 3);

    //cube_file.get_comments().push_back("generated by tinyply 2.2");

    // Write an ASCII file
    //cube_file.write(outstream_ascii, false);

    // Write a binary file
    plyFile.write(outstream_binary, true);

    return true;
}

EncodedData CPC::PointCloudIO::loadCpc(const std::string & inputPath)
{
    // Decompress the huffman encoding first
    // create the decompressed point cloud file
    
    boost::filesystem::path decompressedFilePath(inputPath);
    decompressedFilePath = decompressedFilePath.replace_extension(".dpc"); 
    std::cout << "Decompressing " << inputPath << std::endl;
    //Huffman::Huffman huffman;
    //huffman.decompress(inputPath, decompressedFilePath.string());
    bool success = zipDecompress(inputPath, decompressedFilePath.string());

    EncodedData data;

    std::ifstream inFile(decompressedFilePath.string(), std::ifstream::binary);
    if (!inFile.is_open())
        return data;
    
    // read in the scene bounding box
    readBinary(inFile, data.sceneBoundingBox.min.x());
    readBinary(inFile, data.sceneBoundingBox.min.y());
    readBinary(inFile, data.sceneBoundingBox.min.z());
    readBinary(inFile, data.sceneBoundingBox.max.x());
    readBinary(inFile, data.sceneBoundingBox.max.y());
    readBinary(inFile, data.sceneBoundingBox.max.z());
    // read in the max depth
    readBinary(inFile, data.maxDepth);
    // read in the sub octree depth
    readBinary(inFile, data.subOctreeDepth);
    // read in the size of the encoded data
    size_t dataSize;
    readBinary(inFile, dataSize);

    // allocate the number of nodes
    data.resize(dataSize);
    // read in the whole chunk of encoded data
    inFile.read((char*)data.encodedData.data(), dataSize * sizeof(unsigned char));

    inFile.close();

    // delete the decompressed point cloud file
    //boost::filesystem::remove(decompressedFilePath);

    return data;
}

bool CPC::PointCloudIO::saveCpc(const std::string & outputPath, EncodedData & encodedData)
{
    if (!encodedData.isValid())
        return false;

    // create the decompressed point cloud file
    boost::filesystem::path decompressedFilePath(outputPath);
    decompressedFilePath = decompressedFilePath.replace_extension(".dpc");
    std::ofstream outFile(decompressedFilePath.string(), std::fstream::binary);
    if (!outFile.is_open())
        return false;

    // write the scene bounding box
    // Big Endian 
    writeBinary(outFile, encodedData.sceneBoundingBox.min.x());
    writeBinary(outFile, encodedData.sceneBoundingBox.min.y());
    writeBinary(outFile, encodedData.sceneBoundingBox.min.z());
    writeBinary(outFile, encodedData.sceneBoundingBox.max.x());
    writeBinary(outFile, encodedData.sceneBoundingBox.max.y());
    writeBinary(outFile, encodedData.sceneBoundingBox.max.z());
    // write the max depth
    writeBinary(outFile, encodedData.maxDepth);
    // write the sub octree depth
    writeBinary(outFile, encodedData.subOctreeDepth);
    // write the size of the encoded data
    writeBinary(outFile, encodedData.encodedData.size());
    // Write the encoded data
    outFile.write((char*)encodedData.encodedData.data(), encodedData.encodedData.size() * sizeof(unsigned char));

    outFile.close();

    // Compress using the huffman encoding
    std::cout << "Compressing " << outputPath << std::endl;
    //Huffman::Huffman huffman;
    //bool success = huffman.compress(decompressedFilePath.string(), outputPath);
    bool success = zipCompress(decompressedFilePath.string(), outputPath);

    // delete the decompressed point cloud file
    //boost::filesystem::remove(decompressedFilePath);

    return success;
}

bool CPC::PointCloudIO::zipCompress(const std::string & input, const std::string & output)
{
    std::stringstream command;
    command << "7z.exe a " << output << ' ' << input;
    std::cout << command.str() << std::endl;
    int error = system(command.str().c_str());
    return error == 0;
}

bool CPC::PointCloudIO::zipDecompress(const std::string & input, const std::string & output)
{
    std::stringstream command;
    command << "7z.exe e " << output << ' ' << input << " -y";
    int error = system(command.str().c_str());
    return error == 0;
}
