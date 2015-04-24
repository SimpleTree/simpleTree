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
#include "allometry.h"
namespace simpleTree {
Allometry::Allometry()
{
    coeff_a = 123.6;
    coeff_b = 2.656;
}

void
Allometry::improveTree()
{
    for(size_t i = 0; i  < cylinders.size(); i++)
    {
        boost::shared_ptr<Cylinder> cylinder = cylinders.at(i);
        float x = cylinder->getRadius();
        float y = getTree()->getGrowthVolume ( cylinder );
        if (getXFromY(y)>(x*fac) || getXFromY(y)<(x/fac) )
        {
            cylinder->values[6] = std::max(getXFromY(y),0.025f);
        }
    }
}

float
Allometry::getYFromX(float x)
{
    float y = coeff_a * (pow (x,coeff_b));
    return y;
}

float
Allometry::getXFromY(float y)
{
    float log_x = (log(y/coeff_a))/(coeff_b);

    float x = exp(log_x);
    return x;
}

void
Allometry::getModelData(std::vector<float> & x, std::vector<float> & y,std::vector<float> & x_model, std::vector<float> & y_model,float max_y, float max_x)
{
    x.clear();
    y.clear();
    x_model.clear();
    y_model.clear();
    max_y = std::numeric_limits<float>::min ();
    max_x = std::numeric_limits<float>::min ();

    for ( size_t i = 0; i < cylinders.size (); i++ ) {
        boost::shared_ptr<simpleTree::Cylinder> cylinder = cylinders.at ( i );
        float radius = cylinder->getRadius ();
        if ( radius > max_x ) {
            max_x = radius;
        }
        x.push_back(radius);
        float volume = getTree() ->getGrowthVolume ( cylinder );
        volume = volume /1000.0f;
        y.push_back(volume);
        if ( volume > max_y ) {
            max_y = volume;
        }
    }
    float x_m = 0;
    float y_m = 0;
    for(size_t i = 0; i < 200; i++)
    {
        x_m += (max_x/200.0f);
        y_m += (max_y/200.0f);
        x_model.push_back(x_m);
        y_model.push_back(y_m);
    }
}

void
Allometry::setCoefficients(float a, float b)
{
    this->coeff_a = a;
    this->coeff_b = b;
}


boost::shared_ptr<Tree>
Allometry::getTree()
{
    return this->tree.lock();
}

void
Allometry::setTree(boost::shared_ptr<Tree> tree)
{
    this->tree = tree;
    this->cylinders = tree->getCylinders();

}





}
