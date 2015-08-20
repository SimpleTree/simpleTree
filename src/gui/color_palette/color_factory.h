#ifndef COLOR_DIAMETER_CLASSES_H
#define COLOR_DIAMETER_CLASSES_H

#include "color_palette.h"

class Color_Factory
{


private:
    float thresh1_radius = 0;
    float thresh2_radius = 0.035;
    float thresh3_radius = 0.125;

    int r1 = 153;
    int r2 = 255;
    int r3 = 0;

    int ra = 0;
    int rb = 75;

    int g1 = 0;
    int g2 = 255;
    int g3 = 153;

    int ga = 191;
    int gb = 0;

    int b1 = 0;
    int b2 = 51;
    int b3 = 0;

    int ba = 255;
    int bb = 130;

    int thresh1_order = 0;
    int thresh2_order = 7;
    int  thresh3_order;

public:
    Color_Factory();

    Color
    get_color_from_radius(float radius);

    Color
    get_color_from_branch_order(int order);


    void
    set_thresh1_order(int order)
    {
        if(order < 0)
        {
            order = 0;
        }
        if(order < thresh2_order)
        {
            this->thresh1_order = order;
        }
    }

    void
    set_thresh2_order(int order)
    {
        if(order < 0)
        {
            order = 0;
        }
        if(order > thresh1_order && order < thresh3_order)
        {
            this->thresh2_order = order;
        }
    }
    void
    set_thresh3_order(int order)
    {
        if(order < 0)
        {
            order = 0;
        }
        if(order > thresh2_order)
        {
            this->thresh3_order = order;
        }
    }

    void
    set_thresh1_radius(float radius)
    {
        if(radius < 0)
        {
            radius = 0;
        }
        if(radius < thresh2_radius)
        {
            this->thresh1_radius = radius;
        }
    }

    void
    set_thresh2_radius(float radius)
    {
        if(radius < 0)
        {
            radius = 0;
        }
        if(radius > thresh1_radius && radius < thresh3_radius)
        {
            this->thresh2_radius = radius;
        }
    }

    void
    set_thresh3_radius(float radius)
    {
        if(radius < 0)
        {
            radius = 0;
        }
        if(radius > thresh2_radius)
        {
            this->thresh3_radius = radius;
        }
    }

};

#endif // COLOR_DIAMETER_CLASSES_H
