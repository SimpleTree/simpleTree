#include "color_factory.h"

Color_Factory::Color_Factory()
{
}

Color
Color_Factory::get_color_from_radius(float radius)
{
    if(radius < 0)
    {
        radius = 0;
    }
    if(radius < thresh2_radius)
    {
        float dif1 = thresh2_radius - radius;
        float dif2 = thresh2_radius - thresh1_radius;
        int r = r2 - (dif1/dif2)*(r2 -r1);
        int g = g2 - (dif1/dif2)*(g2 -g1);
        int b = b2 - (dif1/dif2)*(b2 -b1);
        Color color;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    }
    else if(radius > thresh2_radius)
    {
        if(radius >thresh3_radius)
        {
            radius = thresh3_radius;
        }
        float dif1 = radius - thresh2_radius;
        float dif2 = thresh3_radius - thresh2_radius;
        int r = r2 - (dif1/dif2)*(r2 -r3);
        int g = g2 - (dif1/dif2)*(g2 -g3);
        int b = b2 - (dif1/dif2)*(b2 -b3);
        Color color;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;

    }
    else
        return Color();

}

Color
Color_Factory::get_color_from_branch_order(int order)
{
    if(order < 1)
    {
        order = 1;
    }

    if(order > 4)
    {
        order = 4;
    }
    Color color;
    switch(order){
    case 3:
        color.r = 0;
        color.g = 0;
        color.b = 139;
        break;
    case 4:
        color.r = 30;
        color.g = 144;
        color.b = 255;
        break;
    case 2:
        color.r = 255;
        color.g = 0;
        color.b = 255;
        break;
    case 1:
        color.r = 240;
        color.g = 128;
        color.b = 128;
        break;
    case 5:
        color.r = 102;
        color.g = 205;
        color.b = 170;
        break;

    default:
        break;
}

//    float dif1 = order;
//    float dif2 = 7;
//    int r = ra - (dif1/dif2)*(ra -rb);
//    int g = ga - (dif1/dif2)*(ga -gb);
//    int b = ba - (dif1/dif2)*(ba -bb);

//    color.r = r;
//    color.g = g;
//    color.b = b;
    return color;


}


