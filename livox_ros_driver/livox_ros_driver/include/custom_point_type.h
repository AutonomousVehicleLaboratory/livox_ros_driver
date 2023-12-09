#define PCL_NO_PRECOMPILE

#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace pcl
{
    struct EIGEN_ALIGN16 PointXYZILTN    // enforce SSE padding for correct memory alignment
    {
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    union
    {
        float data_c[4];
        struct
        {
            float intensity;            // intensity
            std::uint32_t line;         // line number
            std::uint32_t offset_time;  // time offset
            std::uint32_t tag;          // information bytes about noise
        };
    };
    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZILTN,           // here we assume a XYZ + "test" (as fields)
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, intensity, intensity)
                                (std::uint32_t, line, line)
                                (std::uint32_t, offset_time, offset_time)
                                (std::uint32_t, tag, tag)
)

// int
// main (int argc, char** argv)
// {
//   pcl::PointCloud<MyPointType> cloud;
//   cloud.resize (2);
//   cloud.width = 2;
//   cloud.height = 1;

//   cloud[0].test = 1;
//   cloud[1].test = 2;
//   cloud[0].x = cloud[0].y = cloud[0].z = 0;
//   cloud[1].x = cloud[1].y = cloud[1].z = 3;

//   pcl::io::savePCDFile ("test.pcd", cloud);
// }