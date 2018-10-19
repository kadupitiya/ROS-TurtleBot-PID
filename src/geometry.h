//
// Created by kadupitiya on 9/7/18.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "vector2D.h"

using namespace std;
using namespace geometry_msgs;

class Geometry {

public:

    struct LineSegment {
        VECTOR2D startP;
        VECTOR2D endP;
        double gradient;
        double disatance;
        double disatanceSquared;
        double disatanceToAObj;
        LineSegment *shortestDistLine;

    };

    vector<VECTOR2D> trajectory;
    vector<LineSegment> path;

    map<double, VECTOR2D*> distances;
    map<double, LineSegment*> distancesPath;


    //Member functions

    bool checkInsideRectSimple(VECTOR2D &, VECTOR2D &, VECTOR2D &, VECTOR2D &, VECTOR2D &);

    double crossProduct(VECTOR2D &, VECTOR2D &);

    double dotProduct(VECTOR2D &, VECTOR2D &);

    bool doBoundingBoxesIntersect(LineSegment &, LineSegment &);

    bool isPointOnLine(LineSegment &, VECTOR2D &);

    bool isPointRightOfLine(LineSegment &, VECTOR2D &);

    bool lineSegmentTouchesOrCrossesLine(LineSegment &a, LineSegment &b);

    LineSegment getBoundingBox(LineSegment &lineSeg);

    bool doLinesIntersect(LineSegment &, LineSegment &);

    LineSegment getLineSegment(VECTOR2D &, VECTOR2D &);

    double getDistance(VECTOR2D &p1, VECTOR2D &p2);

    double getDistanceSquared(VECTOR2D &p1, VECTOR2D &p2);

    VECTOR2D* getNearestPoint(VECTOR2D &p);

    int searchAPoint(VECTOR2D *p);

    double getGradient(VECTOR2D &, VECTOR2D &);

    double getMinimumDistancePointToLine(Geometry::LineSegment &l, VECTOR2D &p);

    double getMinimumDistancePointToLineSquared(Geometry::LineSegment &l, VECTOR2D &p);

    LineSegment * getNearestLine(VECTOR2D &p);

    LineSegment getMinimumDistanceLine(Geometry::LineSegment &l, VECTOR2D &p);

    LineSegment * getNextLineSegment(LineSegment* l);

    double correctAngle(double);

};


#endif //GEOMETRY_H
