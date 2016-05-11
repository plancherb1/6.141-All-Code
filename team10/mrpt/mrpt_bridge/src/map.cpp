
#include "mrpt_bridge/map.h"
#include "mrpt_bridge/pose.h"
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

// Only include MRPT classes that are really used to avoid slow down compilation
#include <mrpt/random.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h> // for fileExists()
#include <mrpt/system/string_utils.h> // for lowerCase()

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
#	include <mrpt/maps/COccupancyGridMap2D.h>
#	include <mrpt/maps/CMultiMetricMap.h>
#	include <mrpt/maps/CSimpleMap.h>
	using mrpt::maps::COccupancyGridMap2D;
	using mrpt::maps::CMultiMetricMap;
	using mrpt::maps::CSimpleMap;
	using mrpt::maps::CLogOddsGridMapLUT;
#else
#	include <mrpt/slam/COccupancyGridMap2D.h>
#	include <mrpt/slam/CMultiMetricMap.h>
#	include <mrpt/slam/CSimpleMap.h>
	using mrpt::slam::COccupancyGridMap2D;
	using mrpt::slam::CMultiMetricMap;
	using mrpt::slam::CSimpleMap;
	using mrpt::slam::CLogOddsGridMapLUT;
#endif


#ifndef INT8_MAX // avoid duplicated #define's
#define INT8_MAX    0x7f
#define INT8_MIN    (-INT8_MAX - 1)
#define INT16_MAX   0x7fff
#define INT16_MIN   (-INT16_MAX - 1)
#endif // INT8_MAX

namespace mrpt_bridge
{
MapHdl* MapHdl::instance_ = NULL;

MapHdl::MapHdl ()
{
    /// ceation of the lookup table and pointers
	CLogOddsGridMapLUT<COccupancyGridMap2D::cellType> table;
#ifdef  OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
    lut_cellmrpt2rosPtr = lut_cellmrpt2ros + INT8_MAX + 1; // center the pointer
    lut_cellros2mrptPtr = lut_cellros2mrpt + INT8_MAX + 1; // center the pointer
    for ( int i = INT8_MIN; i < INT8_MAX; i++ ) {
#else
    lut_cellmrpt2rosPtr = lut_cellmrpt2ros + INT16_MAX + 1; // center the pointer
    for ( int i = INT16_MIN; INT16_MIN < INT16_MAX; i++ ) {
#endif
        float p = 1.0-table.l2p ( i );
        int idx = round(p * 100.);
        lut_cellmrpt2rosPtr[i] = idx;
        // printf("- cell -> ros = %4i -> %4i, p=%4.3f\n", i, idx, p);
    }
    for ( int i = INT8_MIN; i < INT8_MAX; i++ ) {
        float v = i;
        if( v > 100) v = 50;
        if( v < 0) v = 50;
        float p = 1.0 - (v /100.0);
        int idx = table.p2l(p);
        if(i < 0)  lut_cellros2mrptPtr[i] = table.p2l(0.5);
        else if(i > 100) lut_cellros2mrptPtr[i] = table.p2l(0.5);
        else lut_cellros2mrptPtr[i] = idx;
        // printf("- ros -> cell = %4i -> %4i, p=%4.3f\n", i, idx, p);
        fflush(stdout);
    }
}
MapHdl::~MapHdl () { }

MapHdl* MapHdl::instance ()
{
    if ( instance_ == NULL ) instance_ = new MapHdl ();
    return instance_;
}

bool convert ( const nav_msgs::OccupancyGrid  &src, COccupancyGridMap2D  &des )
{
    if((src.info.origin.orientation.x != 0) ||
            (src.info.origin.orientation.y != 0) ||
            (src.info.origin.orientation.z != 0) ||
            (src.info.origin.orientation.w != 1)) {
        std::cerr << "Rotated maps are not supported by mrpt!" << std::endl;
        return false;
    }
    float xmin = src.info.origin.position.x;
    float ymin = src.info.origin.position.y;
    float xmax = xmin + src.info.width * src.info.resolution;
    float ymax = ymin + src.info.height * src.info.resolution;

    MRPT_START
    des.setSize(xmin, xmax, ymin, ymax, src.info.resolution);
    MRPT_END
    //printf("--------convert:  %i x %i, %4.3f, %4.3f, %4.3f, %4.3f, r:%4.3f\n",des.getSizeX(), des.getSizeY(), des.getXMin(), des.getXMax(), des.getYMin(), des.getYMax(), des.getResolution());

    /// I hope the data is allways aligned
    for ( int h = 0; h < src.info.height; h++ ) {
		COccupancyGridMap2D::cellType *pDes = des.getRow (h);
        const int8_t *pSrc = &src.data[h * src.info.width];
        for ( int w = 0; w < src.info.width; w++ ) {
            *pDes++ = MapHdl::instance()->cellRos2Mrpt(*pSrc++);
        }
    }
    return true;
}
bool convert ( const COccupancyGridMap2D &src, nav_msgs::OccupancyGrid &des,    const std_msgs::Header &header
)
{
    des.header = header;
    return convert(src, des);
}

bool convert (    const COccupancyGridMap2D &src, nav_msgs::OccupancyGrid &des)
{
    //printf("--------mrpt2ros:  %f, %f, %f, %f, r:%f\n",src.getXMin(), src.getXMax(), src.getYMin(), src.getYMax(), src.getResolution());
    des.info.width = src.getSizeX();
    des.info.height = src.getSizeY();
    des.info.resolution = src.getResolution ();

    des.info.origin.position.x = src.getXMin();
    des.info.origin.position.y = src.getYMin();
    des.info.origin.position.z = 0;

    des.info.origin.orientation.x = 0;
    des.info.origin.orientation.y = 0;
    des.info.origin.orientation.z = 0;
    des.info.origin.orientation.w = 1;

    /// I hope the data is allways aligned
    des.data.resize ( des.info.width*des.info.height );
    for ( int h = 0; h < des.info.height; h++ ) {
		const COccupancyGridMap2D::cellType *pSrc = src.getRow (h);
        int8_t *pDes = &des.data[h * des.info.width];
        for ( int w = 0; w < des.info.width; w++ ) {
            *pDes++ = MapHdl::instance()->cellMrpt2Ros(*pSrc++);
        }
    }
    return true;
}

const bool MapHdl::loadMap(CMultiMetricMap &_metric_map, const mrpt::utils::CConfigFile &_config_file, const std::string &_map_file, const std::string &_section_name, bool _debug) {
#if MRPT_VERSION>=0x130
	using namespace mrpt::maps;
#else
	using namespace mrpt::slam;
#endif

	TSetOfMetricMapInitializers mapInitializers;
    mapInitializers.loadFromConfigFile( _config_file, _section_name);
    if(_debug) mapInitializers.dumpToConsole();

	CSimpleMap  simpleMap;

    // Load the set of metric maps to consider in the experiments:
    _metric_map.setListOfMaps( &mapInitializers );
    if(_debug) mapInitializers.dumpToConsole();

    mrpt::random::randomGenerator.randomize();

    if(_debug) printf("%s, _map_file.size() = %zu\n", _map_file.c_str(), _map_file.size());
    // Load the map (if any):
    if (_map_file.size() < 3) {
        if(_debug) printf("No mrpt map file!\n");
        return false;
    } else {
        ASSERT_( mrpt::system::fileExists(_map_file) );

        // Detect file extension:
        std::string mapExt = mrpt::system::lowerCase( mrpt::system::extractFileExtension( _map_file, true ) ); // Ignore possible .gz extensions

        if ( !mapExt.compare( "simplemap" ) )
        {
            // It's a ".simplemap":
            if(_debug) ("Loading '.simplemap' file...");
            mrpt::utils::CFileGZInputStream(_map_file) >> simpleMap;
            printf("Ok\n");

            ASSERTMSG_( simpleMap.size()>0, "Simplemap was aparently loaded OK, but it is empty!" );

            // Build metric map:
            if(_debug) printf("Building metric map(s) from '.simplemap'...");
            _metric_map.loadFromSimpleMap(simpleMap);
            if(_debug) printf("Ok\n");
        }
        else if ( !mapExt.compare( "gridmap" ) )
        {
            // It's a ".gridmap":
            if(_debug) printf("Loading gridmap from '.gridmap'...");
            ASSERTMSG_( _metric_map.m_gridMaps.size()==1, "Error: Trying to load a gridmap into a multi-metric map requires 1 gridmap member." );
            mrpt::utils::CFileGZInputStream(_map_file) >> (*_metric_map.m_gridMaps[0]);
            if(_debug) printf("Ok\n");
        }
        else
        {
            THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
            return false;
        }

    }
    return true;
}


} // end namespace
