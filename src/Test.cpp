//
// Created by kadupitiya on 10/10/18.
//
#include "pid.h"
#include <stdio.h>
#include "geometry.h"

int main2() {

    PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

    double val = 20;
    for (int i = 0; i < 100; i++) {
        double inc = pid.calculate(0, val);
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val += inc;
    }

    VECTOR2D point1(5,5);
    VECTOR2D point2(10,10);
    VECTOR2D point(10,5);

    Geometry geo;
    Geometry::LineSegment line = geo.getLineSegment(point1,point2);
    Geometry::LineSegment distance = geo.getMinimumDistanceLine(line,point);


    bool pointRight= geo.isPointRightOfLine(line,point);

    printf("Min Distance:% 7.3f Angle:% 7.3f \n", distance.disatance,line.gradient);

    cout << "Point Right: " << pointRight <<endl;


    return 0;



}