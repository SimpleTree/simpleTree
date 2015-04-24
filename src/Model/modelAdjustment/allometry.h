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
#ifndef ALLOMETRY_H
#define ALLOMETRY_H

#include <vector>

#include<math.h>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "../Tree.h"

class Tree;
namespace simpleTree {


class Allometry
{
private:
    float fac = 1.3f;
    float coeff_a, coeff_b;
    boost::weak_ptr<Tree> tree;
    std::vector<float> vec_x;
    std::vector<float> vec_y;
    std::vector<float> model_x;
    std::vector<float> model_y;
    std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders;

    float getYFromX(float x);
    float getXFromY(float y);

public:
    Allometry();

    void
    improveTree();

    boost::shared_ptr<Tree>
    getTree();

    void
    setTree(boost::shared_ptr<Tree> tree);

    void
    setCoefficients(float a, float b);

    void
    getModelData(std::vector<float> & x, std::vector<float> & y,std::vector<float> & x_model, std::vector<float> & y_model,float max_y, float max_x);


};

}
#endif // ALLOMETRY_H
