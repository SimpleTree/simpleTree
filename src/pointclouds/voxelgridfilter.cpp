#include "voxelgridfilter.h"

VoxelGridFilter::VoxelGridFilter()
{
}


void
VoxelGridFilter::voxel_grid_filter () {
    writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    pcl::VoxelGrid<PointI> sor;
    QString str = "";

    str.append ( "Voxel grid filtering starts.\n, all points inside a voxel with " ).append ( QString::number ( voxel_grid_size ) ).append (
        " side length are merged to one point.\n" );
    writeConsole ( str );

    sor.setInputCloud ( getControl ()->getCloudPtr () );
    sor.setLeafSize ( voxel_grid_size, voxel_grid_size, voxel_grid_size );
    sor.filter ( *cloud_filtered );

    int size_before = getControl ()->getCloudPtr ()->points.size ();
    int size_after = cloud_filtered->points.size ();
    float a = size_before;
    float b = size_after;
    float percentage = ( 100.0f * b ) / ( a );
    getControl ()->setCloudPtr ( cloud_filtered );
    writeConsole (
        QString ( "Downsampling done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
            QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );

    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    voxel_grid_size = 0.01;
}
