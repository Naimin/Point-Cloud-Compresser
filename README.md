# Point-Cloud-Compresser
Unstructured Point Cloud Compresser

Description:
This project seek to compress unstructured point cloud file to to provide a more lightweight transfer file while still retaining as much precision as possible.

Usage:
-i / --input : The input file path (this can take in either a .ply or .cpc)
-o / --output : (Optional) The output file path, by default the output file path will be automatically determined by the input file type. (file.ply->file.cpc and file.cpc->file_decoded.ply)
-d / --depth : Define the max depth the octree level should have. This is only used when compressing a .ply file.
-h / --help : Print help information

To compile:
Please use the "download_thirdparty.bat" to download the thirdparty (Boost, Eigen, TBB) used by the project.
