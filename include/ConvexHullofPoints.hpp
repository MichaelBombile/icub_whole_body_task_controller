/** Class ConvexHullofPoints
	class that compute the convex hull formed by several points.
	It requires at least 3 points.

	Given a set of points, it will output the points that form the convex hull
	the normal to the edges of the convex hull
	the distances of the each edges to the origin of points located within the convex hull
*/

#pragma once

#ifndef ConvexHullofPoints_H
#define ConvexHullofPoints_H


#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <bits/stdc++.h> 
using namespace std; 
using namespace Eigen;

// ====================================================================================================
// CLASS ConvexHullofPoints
// ====================================================================================================
struct Point 
{ 
    double x, y; 
}; 

class ConvexHullofPoints
{	
	
	public: 

		//
		int nCHpts;   					// Number of points forming the convex hull
		// Initialize Result 
		// vector<Point> hull; 
		// 
		MatrixXd PointsCHull;			// Matrix containing the points of the convex hull +1
		MatrixXd EdgesNormalCHull;		// Matrix of the normal to the convex hull edges
		VectorXd DistanceEdgesCHull;	// Vector of the distance to the convex hull edges wrt. to center

		ConvexHullofPoints(){};

		~ConvexHullofPoints(){};

		//****************************************************************************************
		// part of this algorithm (orientation() and convexHull()) can be found at :
		// https://www.geeksforgeeks.org/convex-hull-set-1-jarviss-algorithm-or-wrapping/
		//****************************************************************************************

		// To find orientation of ordered triplet (p, q, r). 
		// The function returns following values 
		// 0 --> p, q and r are colinear 
		// 1 --> Clockwise 
		// 2 --> Counterclockwise 
		int orientation(Point p, Point q, Point r) 
		{ 
		    // int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y); 
		    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y); 
		  
		    if (val == 0) return 0;  // colinear 
		    return (val > 0)? 1: 2; // clock or counterclock wise 
		} 
		  
		// Prints convex hull of a set of n points. 
		void convexHull(MatrixXd AllPtsInAbsFoot_, MatrixXd &PointsCHull, MatrixXd &EdgesNormalCHull, VectorXd &DistanceEdgesCHull) 
		{ 
		    // Point points[], int n, 
		    // There must be at least 3 points 
		    int n = AllPtsInAbsFoot_.cols();
		    if (n < 3) return; 

		    // Preparing the point to find the convex hull         // 
    		Point points[n];

		    for (int i=0; i<n; i++) 
		    {
		        // passing to the point struct
		        points[i].x = AllPtsInAbsFoot_(0,i);                                 // x
		        points[i].y = AllPtsInAbsFoot_(1,i);                                 // y 
		    }

		    // Find the leftmost point 
		    int l = 0; 
		    for (int i = 1; i < n; i++) 
		        if (points[i].x < points[l].x) 
		            l = i; 
		  
		    // Start from leftmost point, keep moving counterclockwise 
		    // until reach the start point again.  This loop runs O(h) 
		    // times where h is number of points in result or output. 
		    vector<Point> hull; 

		    int p = l, q; 
		    do
		    { 
		        // Add current point to result 
		        hull.push_back(points[p]); 
		  
		        // Search for a point 'q' such that orientation(p, x, 
		        // q) is counterclockwise for all points 'x'. The idea 
		        // is to keep track of last visited most counterclock- 
		        // wise point in q. If any point 'i' is more counterclock- 
		        // wise than q, then update q. 
		        q = (p+1)%n;
		        for (int i = 0; i < n; i++) 
		        { 
		           // If i is more counterclockwise than current q, then 
		           // update q 
		           if (this->orientation(points[p], points[i], points[q]) == 2) 
		               q = i; 
		        } 
		        // Now q is the most counterclockwise with respect to p 
		        // Set p as q for next iteration, so that q is added to 
		        // result 'hull' 
		        p = q; 
		  
		    } while (p != l);  // While we don't come to first point 
		  
		    // Print Result 
		    // for (int i = 0; i < hull.size(); i++) 
		    //     cout << "(" << hull[i].x << ", " << hull[i].y << ")\n"; 

		    // ==========================================================================
		    //
		    // ==========================================================================

		    // Normals du support polygon edges
		    int nCHpts = hull.size(); //6;
		    MatrixXd PointsCHull_1(nCHpts+1,2);  	// number of rows = nb of points + 1, with last rows equals first row
		    PointsCHull.resize(nCHpts,2);

		    for (int i = 0; i <nCHpts; i++) 
		    {
		        PointsCHull_1(i,0) = hull[i].x;   PointsCHull_1(i,1) = hull[i].y;   // plus one row
		        PointsCHull(i,0) = hull[i].x;     PointsCHull(i,1) = hull[i].y;  
		    }
		    	PointsCHull_1(nCHpts,0) = hull[0].x;   PointsCHull_1(nCHpts,1) = hull[0].y;

		    MatrixXd n1(1,2), n2(1,2); 	// Matrix of the edges normals with opposed directions (inward and outward the convex hull) 

		    EdgesNormalCHull.resize(nCHpts,2);     EdgesNormalCHull.setZero();		// Outward normals to the Convex hulledges
		    DistanceEdgesCHull.resize(nCHpts);     DistanceEdgesCHull.setZero();	// Distance from center (origin of points) to edges of CH

		    for(int i=0; i<nCHpts; i++)
		    {
		        double Dx = PointsCHull_1(i,0) - PointsCHull_1(i+1,0);
		        double Dy = PointsCHull_1(i,1) - PointsCHull_1(i+1,1);
		        //
		        n1 << -Dy, Dx;
		        n2 <<  Dy,-Dx;
				//
		        EdgesNormalCHull.block<1,2>(i,0) = ((n1(0,0)*PointsCHull_1(i,0) + n1(0,1)*PointsCHull_1(i,1)) >=0)? n1 : n2;
		        //
		        DistanceEdgesCHull(i) = fabs( ( PointsCHull_1(i+1,0)*PointsCHull_1(i,1) - PointsCHull_1(i+1,1)*PointsCHull_1(i,0)))/sqrt(Dx*Dx + Dy*Dy);
		    }

		    // cout << " PointsCHull is \n" << PointsCHull << endl;
		    // cout << " EdgesNormalCHull is \n" << EdgesNormalCHull << endl;
		    // cout << " DistanceEdgesCHull is \n" << DistanceEdgesCHull << endl;
		} 

};

#endif // ConvexHullofPoints_H
