#ifndef GUISUBCLASS_H
#define GUISUBCLASS_H

class GuiSubClass
{
public:
    virtual
    ~GuiSubClass()
    {

    }

    virtual void
    setViewer(boost::shared_ptr<PCLViewer> guiPtr) = 0;

    virtual boost::shared_ptr<PCLViewer>
    getViewer() = 0;

    virtual void
    init() = 0;
};



#endif // GUISUBCLASS_H
