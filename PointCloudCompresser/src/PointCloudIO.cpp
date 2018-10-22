#include "PointCloudIO.h"
#include <iostream>
#include <fstream>
#include <chrono>
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

PointCloud CPC::PointCloudIO::loadCpc(const std::string & path)
{


    return PointCloud();
}

bool CPC::PointCloudIO::saveCpc(const std::string & path, EncodedData & encodedData)
{
    if (!encodedData.isValid())
        return false;

    std::ofstream outfile(path, std::ofstream::binary);
    if (!outfile.is_open())
        return false;

    // write the scene bounding box
    // Big Endian 
    writeBinary(outfile, encodedData.sceneBoundingBox.min.x());
    writeBinary(outfile, encodedData.sceneBoundingBox.min.y());
    writeBinary(outfile, encodedData.sceneBoundingBox.min.z());
    writeBinary(outfile, encodedData.sceneBoundingBox.max.x());
    writeBinary(outfile, encodedData.sceneBoundingBox.max.y());
    writeBinary(outfile, encodedData.sceneBoundingBox.max.z());
    // write the max depth
    writeBinary(outfile, encodedData.maxDepth);
    // Write the encoded data
    outfile.write((char*)encodedData.encodedData.data(), encodedData.encodedData.size() * sizeof(unsigned char));

    outfile.close();

    return true;
}