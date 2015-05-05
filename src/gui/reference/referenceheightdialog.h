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

    void
    init();

private:
    /** \brief A weak pointer to the UI class
     * */
    boost::weak_ptr<PCLViewer> viewer;

    float height = 0.2f;



signals:

public slots:
    void
    set_reference_height ( double h );

    void
    compute_reference ();

};

#endif // REFERENCEHEIGHTDIALOG_H
