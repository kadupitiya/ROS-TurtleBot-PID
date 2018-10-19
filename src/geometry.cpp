//
// Created by kadupitiya on 9/7/18.
//

#include "geometry.h"

/*
 * Check if a point is inside a rectangle simple way
 * */
bool Geometry::checkInsideRectSimple(VECTOR2D &p1, VECTOR2D &p2, VECTOR2D &p3, VECTOR2D &p4, VECTOR2D &p) {

    bool x_condition = false;
    bool y_condition = false;

    if (p1.x >= p3.x)
        x_condition = p1.x >= p.x && p.x >= p3.x;
    else
        x_condition = p1.x <= p.x && p.x <= p3.x;

    if (p1.y >= p3.y)
        y_condition = p1.y >= p.y && p.y >= p3.y;
    else
        y_condition = p1.y <= p.y && p.y <= p3.y;

    return x_condition && y_condition;

}

/**
 * Calculate the cross product of two points.
 */
double Geometry::crossProduct(VECTOR2D &a, VECTOR2D &b) {
    return a.x * b.y - b.x * a.y;
}

/**
 * Calculate the dot product of two points.
 */
double Geometry::dotProduct(VECTOR2D &a, VECTOR2D &b) {
    return a.x * b.x + a.y * a.y;
}

/*
 * This function calculate euclidean distance between two given points
 * Remove sqrt if performance hungry
 **/
double Geometry::getDistance(VECTOR2D &p1, VECTOR2D &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

/*
 * This function calculate euclidean distance between two given points
 * Remove sqrt if performance hungry
 **/
double Geometry::getDistanceSquared(VECTOR2D &p1, VECTOR2D &p2) {
    return (pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}



/**
 * Calculate minimum distance between a Line segment and a Point
 */

double Geometry::getMinimumDistancePointToLine(Geometry::LineSegment &l, VECTOR2D &p) {

    // Return minimum distance between line segment l and point p

    double distance = sqrt(getMinimumDistancePointToLineSquared(l, p));
    l.disatanceToAObj = distance;

   return distance;


}


/**
 * Calculate sqaured value of the minimum distance between a Line segment and a Point
 */

double Geometry::getMinimumDistancePointToLineSquared(Geometry::LineSegment &l, VECTOR2D &p) {

    // Return minimum distance between line segment l and point p

    double l_2 = l.disatanceSquared;

    if (l_2 == 0.0) return getDistance(l.startP, p);   // Line doesnt exist

    double px = l.endP.x - l.startP.x;
    double py = l.endP.y - l.startP.y;

    double u = ((p.x - l.startP.x) * px + (p.y - l.startP.y) * py) / l_2;

    if (u > 1)
        u = 1;
    else if (u < 0)
        u = 0;

    double x = l.startP.x + u * px;
    double y = l.startP.y + u * py;

    double dx = x - p.x;
    double dy = y - p.y;

    return (dx * dx + dy * dy);


}

/**
 * Calculate sqaured value of the minimum distance between a Line segment and a Point
 */

Geometry::LineSegment Geometry::getMinimumDistanceLine(Geometry::LineSegment &l, VECTOR2D &p) {

    // Return minimum distance between line segment l and point p

    double l_2 = l.disatanceSquared;

    if (l_2 == 0.0)

        return getLineSegment(l.endP, p);   // Line doesnt exist

    double px = l.endP.x - l.startP.x;
    double py = l.endP.y - l.startP.y;

    double u = ((p.x - l.startP.x) * px + (p.y - l.startP.y) * py) / l_2;

    if (u > 1)
        u = 1;
    else if (u < 0)
        u = 1; // Put u=0 if you need start point

    double x = l.startP.x + u * px;
    double y = l.startP.y + u * py;

    VECTOR2D newPoint(x,y);

    return getLineSegment(p, newPoint);


}


/**
 * Check if bounding boxes do intersect. If one bounding box
 * touches the other, they do intersect.
 * a1 and a2: 1st bounding box
 * b1 and b2: 2nd bounding box
 */
bool Geometry::doBoundingBoxesIntersect(LineSegment &a, LineSegment &b) {
    return a.startP.x <= b.endP.x && a.endP.x >= b.startP.x && a.startP.y <= b.endP.y && a.endP.y >= b.startP.y;
}

/**
 * Checks if a Point is on a line
 */
bool Geometry::isPointOnLine(LineSegment &a, VECTOR2D &b) {
    double EPSILON = 0.000001;
    LineSegment aTmp;
    VECTOR2D tmp1, tmp2;
    tmp1.x = 0.0;
    tmp1.y = 0.0;
    tmp2.x = a.endP.x - a.startP.x;
    tmp2.y = a.endP.y - a.startP.y;
    aTmp.startP = tmp1;
    aTmp.endP = tmp2;
    VECTOR2D bTmp;
    bTmp.x = b.x - a.startP.x;
    bTmp.y = b.y - a.startP.y;
    double r = crossProduct(aTmp.endP, bTmp);
    return abs(r) < EPSILON;
}

/**
 * Checks if a point is right of a line. If the point is on the
 * line, it is not right of the line.
 */
bool Geometry::isPointRightOfLine(LineSegment &a, VECTOR2D &b) {
    LineSegment aTmp;
    VECTOR2D tmp1, tmp2;
    tmp1.x = 0.0;
    tmp1.y = 0.0;
    tmp2.x = a.endP.x - a.startP.x;
    tmp2.y = a.endP.y - a.startP.y;
    aTmp.startP = tmp1;
    aTmp.endP = tmp2;
    VECTOR2D bTmp;
    bTmp.x = b.x - a.startP.x;
    bTmp.y = b.y - a.startP.y;
    return crossProduct(aTmp.endP, bTmp) < 0;
}

/**
 * Check if line segment first touches or crosses the line that is
 * defined by line segment second.
 */
bool Geometry::lineSegmentTouchesOrCrossesLine(LineSegment &a, LineSegment &b) {
    return isPointOnLine(a, b.startP)
           || isPointOnLine(a, b.endP)
           || (!isPointRightOfLine(a, b.startP) != !isPointRightOfLine(a, b.endP));
}

/**
 * Get the bounding box of this line by two points. The first point is in
 * the lower left corner, the second one at the upper right corner.
 */
Geometry::LineSegment Geometry::getBoundingBox(LineSegment &lineSeg) {
    VECTOR2D p1, q1;
    p1.x = min(lineSeg.startP.x, lineSeg.endP.x);
    p1.y = min(lineSeg.startP.y, lineSeg.endP.y);
    q1.x = max(lineSeg.startP.x, lineSeg.endP.x);
    q1.y = max(lineSeg.startP.y, lineSeg.endP.y);

    LineSegment box_bounds;
    box_bounds.startP = p1;
    box_bounds.endP = q1;

    return box_bounds;
}


/**
 * Check if line segments intersect
 * a first line segment
 * b second line segment
 */
bool Geometry::doLinesIntersect(LineSegment &a, LineSegment &b) {
    LineSegment box1 = getBoundingBox(a);
    LineSegment box2 = getBoundingBox(b);
    return doBoundingBoxesIntersect(box1, box2)
           && lineSegmentTouchesOrCrossesLine(a, b)
           && lineSegmentTouchesOrCrossesLine(b, a);
}

/*
 * Returns a Line segment for a given two points
 */
Geometry::LineSegment Geometry::getLineSegment(VECTOR2D &a, VECTOR2D &b) {
    /*direction vector a->b*/
    LineSegment aTmp;
    aTmp.startP = a;
    aTmp.endP = b;
    aTmp.gradient = getGradient(a, b);
    aTmp.disatance = getDistance(a, b);
    aTmp.disatanceSquared = getDistanceSquared(a, b);
    return aTmp;
}


/*
 * Return the nearest node pointer to a given point
 **/
VECTOR2D *Geometry::getNearestPoint(VECTOR2D &p) {

    distances.clear();

    int size = trajectory.size();
    if (size == 1) {
        return (&trajectory[0]);
    } else {
        /*Enhance this later for better realtime perf for large systems*/

        for (int i = 0; i < size; i++)
            distances[getDistance(p, trajectory[i])] = &trajectory[i];

        //second means value [key, value]
        return distances.begin()->second;

    }
}

/*
 * Return the nearest Linesegment to a given point
 **/
Geometry::LineSegment *Geometry::getNearestLine(VECTOR2D &p) {

    distancesPath.clear();

    int size = path.size();
    if (size == 1) {
        return (&path[0]);
    } else {
        /*Enhance this later for better realtime perf for large systems*/

        for (int i = 0; i < size; i++)
            distancesPath[getMinimumDistancePointToLine(path[i], p)] = &path[i];

        //second means value [key, value]
        return distancesPath.begin()->second;

    }
}


/**
     *
     * This returns a the gradient of a given two points
     */
double Geometry::getGradient(VECTOR2D &p1, VECTOR2D &p2) {
    /*direction vector p1->p2*/
    double grad = 0.0;
    double deltaX = p2.x - p1.x;
    double deltaY = p2.y - p1.y;

    if (deltaX != 0) {

        grad = atan(deltaY / deltaX); //1st region is default

        if (deltaY == 0 && deltaX < 0)//deltaY zero case and deltaX < 0
            grad = M_PI;
        else if (deltaY > 0 && deltaX < 0)// 2nd region
            grad = M_PI + grad;
        else if (deltaY < 0 && deltaX < 0)//3rd region
            grad = -M_PI + grad;
        else if (deltaY < 0 && deltaX > 0)// 4th region
            grad = grad;
        //grad += M_PI;

    } else {
        //deltaX zero case
        if (deltaY > 0)
            grad = M_PI / 2;
        else if (deltaY < 0)
            grad = -M_PI / 2;
        else
            grad = 0.0;
    }

    return grad;
}


/**
     *
     * This returns a the index of the point
     */
int Geometry::searchAPoint(VECTOR2D *p) {

    int size = trajectory.size();
    if (size == 1) {
        return 0;
    } else {
        /*Enhance this later for better realtime perf for large systems*/
        for (int i = 0; i < size; i++)
            if (&trajectory[i] == p)
                return i;

    }

    return -1;
}


/*
 * Return the nearest node pointer to a given point
 **/
Geometry::LineSegment *Geometry::getNextLineSegment(Geometry::LineSegment* l) {


    int size = path.size();
    if (size == 1) {
        return (&path[0]);
    } else {
        /*Enhance this later for better realtime perf for large systems*/
        int i = 0;
        int index=-1;
        for (i = 0; i < size; i++)
            if(&path[i]==l)
                index=i+1;

        //Fix for the last element: Get the 1st element
        if(index==size)
            index=0;

        //second means value [key, value]
        return &path[index];

    }
}

double Geometry::correctAngle(double angle){
    if(angle > M_PI){

        angle= angle - 2*M_PI;
    }

    else if(angle<-M_PI){

        angle= angle + 2*M_PI;
    }
    return angle;
}