/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Jan Hackenberg, University of Freiburg.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/
#include "importpcd.h"

ImportPCD::ImportPCD (std::string fileName,
                      boost::weak_ptr<Controller> control)
{

    this->_control = control;
    this->_fileName = fileName;
    if (fileName.length () > 4)
    {
        std::string ext = fileName.substr (fileName.length () - 3, 3);
        if (ext == "pcd")
        {
            getControl ()->getGuiPtr ()->updateProgress (0);
            QCoreApplication::processEvents ();
            _cloud_intens = importPCD ();
            getControl ()->getGuiPtr ()->updateProgress (100);
            QCoreApplication::processEvents ();
        }
        else if (ext == "asc"||ext == "txt")
        {
            getControl ()->getGuiPtr ()->updateProgress (0);
            QCoreApplication::processEvents ();
            _cloud_intens = importASC ();
            getControl ()->getGuiPtr ()->updateProgress (100);
            QCoreApplication::processEvents ();
        }
    }
}

ImportPCD::ImportPCD (QString fileName)
{

    this->_fileName = fileName.toStdString();
}

void
ImportPCD::compute()
  {
    if (_fileName.length () > 4)
    {
        std::string ext = _fileName.substr (_fileName.length () - 3, 3);
        if (ext == "pcd")
        {
            emit emitProgress(0);
            _cloud_intens = importPCD ();
            emit emitProgress(100);
        }
        else if (ext == "asc"||ext == "txt")
        {
            emit emitProgress(0);
            _cloud_intens = importASC ();
            emit emitProgress(100);
        }
    }
    emit emitCloud(_cloud_intens,true);
}

boost::shared_ptr<Controller>
ImportPCD::getControl ()
{
    return this->_control.lock ();
}

ImportPCD::~ImportPCD ()
{
    // TODO Auto-generated destructor stub
}



pcl::PointCloud<PointI>::Ptr
ImportPCD::importASC ()
{

    tt.tic ();
    pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
    PointCloudI::Ptr temp (new PointCloudI);
    std::ifstream file (_fileName.c_str ());
    std::string line;
    if (file.is_open ())
    {
        while (std::getline (file, line))
        {
            QString qLine = QString::fromStdString (line);
            qLine.replace(QRegExp("[\\s]+"), " ");
            if(qLine.endsWith(" "))
            {
                qLine.chop(1);
            }
            QStringList fields = qLine.split (" ");
            PointI p = toPoint (fields);
            temp->push_back (p);
        }
    }


    QString str;
    float f = tt.toc () / 1000;
    str.append ("Imported ").append (QString::fromStdString (_fileName)).append (" with ").append (QString::number (temp->points.size ())).append (
                " points in  ").append (QString::number (f)).append (" seconds.\n");
    emit emitQString(str,true,true);
    return temp;
}

PointI
ImportPCD::toPoint (QStringList fields)
{
    PointI p;
    if (fields.size () == 3)
    {
        float x, y, z;
        int i = 120;
        QString str = fields.at (0);
        x = str.toFloat ();
        str = fields.at (1);
        y = str.toFloat ();
        str = fields.at (2);
        z = str.toFloat ();
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = i;
    }
    else if (fields.size () == 4)
    {
        float x, y, z;
        int i = 120;
        QString str = fields.at (0);
        x = str.toFloat ();
        str = fields.at (1);
        y = str.toFloat ();
        str = fields.at (2);
        z = str.toFloat ();
        str = fields.at (3);
        i = str.toInt ();
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = i;

    }
    else if (fields.size () == 6)
    {
        float x, y, z;
        int i, r, g, b;
        QString str = fields.at (0);
        x = str.toFloat ();
        str = fields.at (1);
        y = str.toFloat ();
        str = fields.at (2);
        z = str.toFloat ();
        str = fields.at (3);
        r = str.toInt ();
        str = fields.at (4);
        g = str.toInt ();
        str = fields.at (5);
        b = str.toInt ();
        i = std::min<int> (255, ( (r + g + b) / 3));
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = i;

    }
    return p;
}

PointCloudI::Ptr
ImportPCD::importPCD ()
{
    tt.tic ();
    pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
    PointCloudI::Ptr temp (new PointCloudI);

    if (pcl::io::loadPCDFile<PointI> (_fileName, *temp) == -1)
    {
        PCL_ERROR("Couldn't read pcd input file. \n");

    }
    else
    {
        QString str;
        float f = tt.toc () / 1000;
        str.append ("Imported ").append (QString::fromStdString (_fileName)).append (" with ").append (QString::number (temp->points.size ())).append (
                    " points in  ").append (QString::number (f)).append (" seconds.\n");
        emit emitQString(str,true,true);

    }
    return temp;
}



