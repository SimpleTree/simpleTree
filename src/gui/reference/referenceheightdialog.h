#ifndef REFERENCEHEIGHTDIALOG_H
#define REFERENCEHEIGHTDIALOG_H

#include <QDialog>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "../../../src/gui/pclviewer.h"
#include "../../../build/ui_reference_dialog.h"
#include "../../../src/gui/guisubclass.h"

class ReferenceHeightDialog : public QDialog, public GuiSubClass
{
    Q_OBJECT
public:
    /** \brief Default contrstructor
     * \param parent: The parent QT class (main UI class)
     * */
    explicit ReferenceHeightDialog(QWidget *parent = 0);

    /** \brief connects this with the main UI class
     * */
    void
    setViewer(boost::shared_ptr<PCLViewer> guiPtr);

    /** \brief returns the main UI class as a shared pointer
     * \return the shared pointer of main UI class
     * */
    boost::shared_ptr<PCLViewer>
    getViewer();

    /** \brief connects the dialog buttons
     * */
    void
    init();

private:
    /** \brief A weak pointer to the UI class
     * */
    boost::weak_ptr<PCLViewer> viewer;

    /** \brief height to which the minimum z-coordinate of the point cloud should be set to
     * */
    float height = 0.2f;



signals:

public slots:
    /** \brief sets the height
     * \param h: the height
     * */
    void
    set_reference_height ( double h );

    /** \brief performs a translation along the z-axis to set the minium z-Coordinate of the
     * point cloud to the height
     * */
    void
    compute_reference ();

};

#endif // REFERENCEHEIGHTDIALOG_H
