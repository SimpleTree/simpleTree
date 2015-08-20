#include "ery_denoise.h"


Ery_denoise::Ery_denoise(boost::shared_ptr<PCLViewer> viewer, QWidget * parent)
{
    this->viewer = viewer;
}

Ery_denoise::~Ery_denoise()
{

}



void
Ery_denoise::compute()
{
    QString path;
    QString file;
    QStringList files;

    pcl::console::TicToc tt,tt2;
    tt.tic ();
    tt2.tic();
    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                 "../data/",
                                                 QFileDialog::ShowDirsOnly
                                                 | QFileDialog::DontResolveSymlinks | QFileDialog::DontUseNativeDialog);

    QFileInfo fi(dir, file);


    QStringList filters;
    filters << "*.pcd" ;
    dir.setNameFilters ( filters );
    dir.setFilter ( QDir::Files );
    files = dir.entryList ();
    qDebug() << files << "\n";
    path = dir.absolutePath();


    for ( int i = 0; i < files.size (); i++ ) {

        QString file_abs = path ;
        file_abs = file_abs.append ( "/" ).append ( files.at ( i ) );
        qDebug() << "The path to the file" << file_abs;

        int index = file_abs.lastIndexOf ( "/" );
        int size = file_abs.size ();
        int position = size - index - 1;
        file = file_abs.right ( position );
        qDebug() << "The file" << file <<"\n";
        emit emitTreeID(file);
        emit emitQString(file, false, false);


        std::string file_str = file_abs.toStdString ();
        std::string abort;
        if ( file_str != abort )
        {
            ImportPCD import ( file_abs);
            import.compute();
            _cloud_Ptr = import.getCloud();
            emit emitCloud(_cloud_Ptr,true);
        }

        {
            PointCloudI::Ptr cloud_filtered ( new PointCloudI );
            QString str = "";

            str.append ( "Voxel grid filtering starts.\n, all points inside a voxel with " ).append ( QString::number ( voxel_grid_size ) ).append (
                        " side length are merged to one point.\n" );
            emit emitQString( str );

            VoxelGridFilter voxel(0.005,1);
            voxel.setInput(_cloud_Ptr);
            voxel.voxel_grid_filter();
            cloud_filtered = voxel.getOutput();
            {
                int size_before = _cloud_Ptr->points.size ();
                int size_after = cloud_filtered->points.size ();
                float a = size_before;
                float b = size_after;
                float percentage = ( 100.0f * b ) / ( a );
                emit emitQString ( QString ( "Downsampling done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
                                       QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );
            }
            _cloud_Ptr = cloud_filtered;
            emit emitCloud(_cloud_Ptr);
        }

        {

            int euclidean_clustering_minsize = 100;
            float euclidean_clustering_tolerance = 0.02;
            int euclidean_clustering_clusternumber = 1;

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::search::KdTree<PointI>::Ptr tree ( new pcl::search::KdTree<PointI> );
            tree->setInputCloud ( _cloud_Ptr );
            pcl::EuclideanClusterExtraction<PointI> ec;
            ec.setClusterTolerance ( euclidean_clustering_tolerance ); // 2cm
            ec.setMinClusterSize ( euclidean_clustering_minsize );
            ec.setSearchMethod ( tree );
            ec.setInputCloud ( _cloud_Ptr );
            ec.extract ( cluster_indices );
            PointCloudI::Ptr cloud_filtered ( new PointCloudI );
            if ( cluster_indices.size () > 0 ) {
                int i = std::min<int> ( euclidean_clustering_clusternumber, cluster_indices.size () );
                if(euclidean_clustering_clusternumber == 1)
                {
                    pcl::PointIndices largestCluster = cluster_indices.at(0);
                    size_t largest_cluster_size = largestCluster.indices.size();
                    float center_z = std::numeric_limits<float>::max();
                    size_t j = 0;
                    while(j<cluster_indices.size()&&cluster_indices.at(j).indices.size()>largest_cluster_size/3)
                    {
                        PointCloudI::Ptr tempCloud (new PointCloudI);
                        largestCluster= cluster_indices.at ( j );
                        for ( std::vector<int>::const_iterator pit = largestCluster.indices.begin (); pit != largestCluster.indices.end (); pit++ )
                            tempCloud->points.push_back ( _cloud_Ptr->points[*pit] ); //*
                        Eigen::Vector4f xyz_centroid;
                        pcl::compute3DCentroid<PointI> (*tempCloud, xyz_centroid);
                        float z  = xyz_centroid[2];
                        if(z<center_z)
                        {
                            center_z = z;
                            cloud_filtered = tempCloud;
                        }
                        j++;

                    }
                }
                else
                {
                    for ( int j = 0; j < i; j++ ) {
                        pcl::PointIndices largestCluster;
                        largestCluster= cluster_indices.at ( j );
                        for ( std::vector<int>::const_iterator pit = largestCluster.indices.begin (); pit != largestCluster.indices.end (); pit++ )
                            cloud_filtered->points.push_back ( _cloud_Ptr->points[*pit] ); //*
                    }
                }
            }

            int size_after = cloud_filtered->points.size ();
            cloud_filtered->width = size_after;
            cloud_filtered->height = 1;
            _cloud_Ptr = cloud_filtered;
            emit emitCloud(_cloud_Ptr);
        }

        {
            float  height = 0.1;
            PointCloudI::Ptr cloud = _cloud_Ptr;
            Eigen::Vector4f min_pt;
            Eigen::Vector4f max_pt;
            pcl::getMinMax3D ( *cloud, min_pt, max_pt );
            float oldMinHeight = ( min_pt[2] );
            for ( size_t i = 0; i < cloud->points.size (); i++ ) {
                cloud->points.at ( i ).z = cloud->points.at ( i ).z + height - oldMinHeight;
            }
            _cloud_Ptr = cloud;
            emit emitCloud(_cloud_Ptr);


        }


        QCoreApplication::processEvents ();
        {
            file_abs = path ;
            file_abs = file_abs.append ( "/" ).append ( files.at ( i ) ).append("_automatic_denoised.pcd");

            pcl::io::savePCDFileASCII (file_abs.toStdString(), *_cloud_Ptr);

        }
    }
    QString timestr("running complete folder took ");
    timestr.append(QString::number(tt2.toc()/1000)).append(QString(" seconds.\n"));
    emit emitQString(timestr);
}
