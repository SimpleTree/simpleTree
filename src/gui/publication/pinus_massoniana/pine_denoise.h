#ifndef PINE_DENOISE_H
#define PINE_DENOISE_H

#include "../../../includes.h"
#include "../../../Model/Tree.h"


class CurvatureDialog;


class Pine_denoise : public QDialog
{
        Q_OBJECT


public:
    /** \brief Default contrstructor
     * \param parent: The parent QT class (main UI class)
     * */
    Pine_denoise(boost::shared_ptr<PCLViewer> viewer, QWidget * parent = 0);

    ~Pine_denoise();

    /**
     * @brief the method to calculate all the models for one
     * specific folder with thresholds set to the ones used
     * in the publication
     */
    void
    compute();

signals:
    /**
     * @brief emitCloud emits the point cloud
     * @param cloud the cloud
     */
    void
    emitCloud(PointCloudI::Ptr cloud, bool updateVisu = false);


    /**
     * @brief emitQString emits A QString
     * @param str the emitted String
     * @param firstLine set true if you want to print a line above str
     * @param secondLine set true if you want to print a line below str
     */
    void
    emitQString(QString str = "", bool firstLine = false, bool secondLine = false);

    /**
     * @brief emitTreeID emits the QString of the treeID
     * @param ID the emitted Tree ID
     */
    void
    emitTreeID(QString ID = "");

    /**
     * @brief emitProgress The progress emitted in a percentage number
     * @param progress The progress in percentage
     */
    void
    emitProgress(int progress);

private:
    boost::shared_ptr<PCLViewer> viewer;

        boost::shared_ptr<CurvatureDialog> curvature;

    /**
     * @brief _cloud_Ptr
     */
    PointCloudI::Ptr _cloud_Ptr;

    /**
     * @brief voxel_grid_size The size of a voxel cell
     */
    float voxel_grid_size = 0.01;


    /**
     * @brief _result_string A string to store result text in
     */
    QString _result_string;
};
#endif // PINE_DENOISE_H
