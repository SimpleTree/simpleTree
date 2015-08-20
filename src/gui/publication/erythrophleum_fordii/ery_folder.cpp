#include "ery_folder.h"


Ery_folder::Ery_folder(QWidget * parent)
{

}

Ery_folder::~Ery_folder()
{

}


void
Ery_folder::setStartCoefficient()
{
    _method_coefficients.name = "Ery";
    _method_coefficients.sphere_radius_multiplier = 1.5f;
    _method_coefficients.epsilon_cluster_branch = 0.02f;
    _method_coefficients.epsilon_cluster_stem = 0.03f;
    _method_coefficients.epsilon_sphere = 0.02f;
    _method_coefficients.minPts_ransac_stem = 10;
    _method_coefficients.minPts_ransac_branch = 10;
    _method_coefficients.minPts_cluster_stem = 3;
    _method_coefficients.minPts_cluster_branch = 3;
    _method_coefficients.min_radius_sphere_stem = 0.07f;
    _method_coefficients.min_radius_sphere_branch = 0.025f;

    _method_coefficients.a = 95.551;
    _method_coefficients.b = 2.500;
    _method_coefficients.fact = 2;
    _method_coefficients.minRad = 0.02f;

    _method_coefficients.max_iterations = 6;
    _method_coefficients.seeds_per_voxel = 81;
    _method_coefficients.min_dist =0.0001;

}

void
Ery_folder::compute()
{
    QString path;
    QString file;
    QStringList files;
    QString error;

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
    qDebug() << "The path to the file" << path << "\n";

    for ( int i = 0; i < files.size (); i++ )
    {
        setStartCoefficient();
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
        if ( file_str != abort ) {
            ImportPCD import ( file_abs);
            import.compute();
            _cloud_Ptr = import.getCloud();
            emit emitCloud(_cloud_Ptr,true);
        }

        std::vector<float> e1;
        std::vector<float> e2;
        std::vector<float> e3;
        std::vector<bool> isStem;

        EigenValueEstimator es (_cloud_Ptr , e1, e2, e3, isStem, 0.035f );
        StemPointDetection detect ( _cloud_Ptr, isStem );
        isStem = detect.getStemPtsNew ();
        emit emitE1(e1);
        emit emitE2(e2);
        emit emitE3(e3);
        emit emitIsStem(isStem);
        emit emitProgress(100);
        QString str;
        float f = tt.toc () / 1000;
        str.append ( "Computed PCA analysis in  " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
        emit emitQString(str,false,false);
        boost::shared_ptr<Optimization> optimize (new Optimization(_method_coefficients.max_iterations, _method_coefficients.seeds_per_voxel, _method_coefficients.min_dist));
        optimize->setCoefficients(_method_coefficients);
        optimize->setCloudPtr(_cloud_Ptr);
        optimize->setIsStem(isStem);
        optimize->setTreeID(files.at(i).toStdString());
        optimize->optimize();
        _method_coefficients = optimize->getCoefficients();
        emit emitQString(_method_coefficients.toQString());



        {
            if (_cloud_Ptr != 0 ) {
                pcl::console::TicToc tt;
                tt.tic ();
                QString str = "\n";

                emit emitProgress(0);
                SphereFollowing sphereFollowing ( _cloud_Ptr, isStem, 1, _method_coefficients );
                emit emitProgress(50);
                boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), _cloud_Ptr,
                                                                                                  files.at(i).toStdString(), true );
                simpleTree::Allometry allom;
                allom.setTree(tree);
                allom.setCoefficients(_method_coefficients.a,_method_coefficients.b);
                allom.setFac(_method_coefficients.fact);
                allom.setMinRad(_method_coefficients.minRad);
                allom.improveTree();

                float f = tt.toc () / 1000;
                str.append ( "Done tree structure in " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
                emit emitQString(str,false,true);
                emit emitProgress(100);
                emit emitTree(tree);


                if ( tree != 0 ) {
                    QString ID =files.at(i);
                    ID = ID.append("_manual_test");
                    ExportPly tree_ply ( tree->getCylinders (), ID.toStdString(), "tree" );
                    WriteCSV write ( tree, ID.toStdString());
                    error.append(QString::number(1)).append(",").append(files.at(i)).append(",").append(QString::number(tree->getVolume())).append(",").append(QString::number(tree->getDBH())).append(",").append(QString::number(tree->getBaseDiameter())).append("\n");
                }

            }
        }
    }




    QString timestr("running complete folder took ");
    timestr.append(QString::number(tt2.toc()/1000)).append(QString(" seconds.\n"));
    emit emitQString(timestr);
    emit emitQString(error);
}




