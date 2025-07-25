
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_distance_calculations.h:

Program Listing for File distance_calculations.h
================================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_distance_calculations.h>` (``hateb_local_planner/include/hateb_local_planner/distance_calculations.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016,
    *  TU Dortmund - Institute of Control Theory and Systems Engineering.
    *  All rights reserved.
    *
    *  Redistribution and use in source and binary forms, with or without
    *  modification, are permitted provided that the following conditions
    *  are met:
    *
    *   * Redistributions of source code must retain the above copyright
    *     notice, this list of conditions and the following disclaimer.
    *   * Redistributions in binary form must reproduce the above
    *     copyright notice, this list of conditions and the following
    *     disclaimer in the documentation and/or other materials provided
    *     with the distribution.
    *   * Neither the name of the institute nor the names of its
    *     contributors may be used to endorse or promote products derived
    *     from this software without specific prior written permission.
    *
    *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    *  POSSIBILITY OF SUCH DAMAGE.
    *
    * Author: Christoph Rösmann
    * Minor Modifications by: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef DISTANCE_CALCULATIONS_H
   #define DISTANCE_CALCULATIONS_H
   
   #include <hateb_local_planner/misc.h>
   
   #include <Eigen/Core>
   #include <algorithm>
   
   namespace hateb_local_planner {
   
   using Point2dContainer = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
   
   inline Eigen::Vector2d closest_point_on_line_segment_2d(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start,
                                                           const Eigen::Ref<const Eigen::Vector2d>& line_end) {
     Eigen::Vector2d diff = line_end - line_start;
     double sq_norm = diff.squaredNorm();
   
     if (sq_norm == 0) {
       return line_start;
     }
   
     double u = ((point.x() - line_start.x()) * diff.x() + (point.y() - line_start.y()) * diff.y()) / sq_norm;
   
     if (u <= 0) {
       return line_start;
     }
     if (u >= 1) {
       return line_end;
     }
   
     return line_start + u * diff;
   }
   
   inline double distance_point_to_segment_2d(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end) {
     return (point - closest_point_on_line_segment_2d(point, line_start, line_end)).norm();
   }
   
   inline bool check_line_segments_intersection_2d(const Eigen::Ref<const Eigen::Vector2d>& line1_start, const Eigen::Ref<const Eigen::Vector2d>& line1_end,
                                                   const Eigen::Ref<const Eigen::Vector2d>& line2_start, const Eigen::Ref<const Eigen::Vector2d>& line2_end, Eigen::Vector2d* intersection = NULL) {
     // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
     double s_numer;
     double t_numer;
     double denom;
     double t;
     Eigen::Vector2d line1 = line1_end - line1_start;
     Eigen::Vector2d line2 = line2_end - line2_start;
   
     denom = line1.x() * line2.y() - line2.x() * line1.y();
     if (denom == 0) {
       return false;
     }  // Collinear
     bool denom_positive = denom > 0;
   
     Eigen::Vector2d aux = line1_start - line2_start;
   
     s_numer = line1.x() * aux.y() - line1.y() * aux.x();
     if ((s_numer < 0) == denom_positive) {
       return false;
     }  // No collision
   
     t_numer = line2.x() * aux.y() - line2.y() * aux.x();
     if ((t_numer < 0) == denom_positive) {
       return false;
     }  // No collision
   
     if (((s_numer > denom) == denom_positive) || ((t_numer > denom) == denom_positive)) {
       return false;
     }  // No collision
   
     // Otherwise collision detected
     t = t_numer / denom;
     if (intersection) {
       *intersection = line1_start + t * line1;
     }
   
     return true;
   }
   
   inline double distance_segment_to_segment_2d(const Eigen::Ref<const Eigen::Vector2d>& line1_start, const Eigen::Ref<const Eigen::Vector2d>& line1_end,
                                                const Eigen::Ref<const Eigen::Vector2d>& line2_start, const Eigen::Ref<const Eigen::Vector2d>& line2_end) {
     // TODO(unknown): more efficient implementation
   
     // check if segments intersect
     if (check_line_segments_intersection_2d(line1_start, line1_end, line2_start, line2_end)) {
       return 0;
     }
   
     // check all 4 combinations
     std::array<double, 4> distances;
   
     distances[0] = distance_point_to_segment_2d(line1_start, line2_start, line2_end);
     distances[1] = distance_point_to_segment_2d(line1_end, line2_start, line2_end);
     distances[2] = distance_point_to_segment_2d(line2_start, line1_start, line1_end);
     distances[3] = distance_point_to_segment_2d(line2_end, line1_start, line1_end);
   
     return *std::min_element(distances.begin(), distances.end());
   }
   
   inline double distance_point_to_polygon_2d(const Eigen::Vector2d& point, const Point2dContainer& vertices) {
     double dist = HUGE_VAL;
   
     // the polygon is a point
     if (vertices.size() == 1) {
       return (point - vertices.front()).norm();
     }
   
     // check each polygon edge
     for (int i = 0; i < static_cast<int>(vertices.size()) - 1; ++i) {
       double new_dist = distance_point_to_segment_2d(point, vertices.at(i), vertices.at(i + 1));
       //       double new_dist = calc_distance_point_to_segment( position,  vertices.at(i), vertices.at(i+1));
       dist = std::min(new_dist, dist);
     }
   
     if (vertices.size() > 2)  // if not a line close polygon
     {
       double new_dist = distance_point_to_segment_2d(point, vertices.back(), vertices.front());  // check last edge
       if (new_dist < dist) {
         return new_dist;
       }
     }
   
     return dist;
   }
   
   inline double distance_segment_to_polygon_2d(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, const Point2dContainer& vertices) {
     double dist = HUGE_VAL;
   
     // the polygon is a point
     if (vertices.size() == 1) {
       return distance_point_to_segment_2d(vertices.front(), line_start, line_end);
     }
   
     // check each polygon edge
     for (int i = 0; i < static_cast<int>(vertices.size()) - 1; ++i) {
       double new_dist = distance_segment_to_segment_2d(line_start, line_end, vertices.at(i), vertices.at(i + 1));
       //       double new_dist = calc_distance_point_to_segment( position,  vertices.at(i), vertices.at(i+1));
       dist = std::min(new_dist, dist);
     }
   
     if (vertices.size() > 2)  // if not a line close polygon
     {
       double new_dist = distance_segment_to_segment_2d(line_start, line_end, vertices.back(), vertices.front());  // check last edge
       if (new_dist < dist) {
         return new_dist;
       }
     }
   
     return dist;
   }
   
   inline double distance_polygon_to_polygon_2d(const Point2dContainer& vertices1, const Point2dContainer& vertices2) {
     double dist = HUGE_VAL;
   
     // the polygon1 is a point
     if (vertices1.size() == 1) {
       return distance_point_to_polygon_2d(vertices1.front(), vertices2);
     }
   
     // check each edge of polygon1
     for (int i = 0; i < static_cast<int>(vertices1.size()) - 1; ++i) {
       double new_dist = distance_segment_to_polygon_2d(vertices1[i], vertices1[i + 1], vertices2);
       dist = std::min(new_dist, dist);
     }
   
     if (vertices1.size() > 2)  // if not a line close polygon1
     {
       double new_dist = distance_segment_to_polygon_2d(vertices1.back(), vertices1.front(), vertices2);  // check last edge
       if (new_dist < dist) {
         return new_dist;
       }
     }
   
     return dist;
   }
   
   // Further distance calculations:
   
   // The Distance Calculations are mainly copied from http://geomalgorithms.com/a07-_distance.html
   // Copyright 2001 softSurfer, 2012 Dan Sunday
   // This code may be freely used and modified for any purpose
   // providing that this copyright notice is included with it.
   // SoftSurfer makes no warranty for this code, and cannot be held
   // liable for any real or imagined damage resulting from its use.
   // Users of this code must verify correctness for their application.
   
   inline double calc_distance_line_to_line_3d(const Eigen::Ref<const Eigen::Vector3d>& x1, Eigen::Ref<const Eigen::Vector3d>& u, const Eigen::Ref<const Eigen::Vector3d>& x2,
                                               Eigen::Ref<const Eigen::Vector3d>& v) {
     Eigen::Vector3d w = x2 - x1;
     double a = u.squaredNorm();  // dot(u,u) always >= 0
     double b = u.dot(v);
     double c = v.squaredNorm();  // dot(v,v) always >= 0
     double d = u.dot(w);
     double e = v.dot(w);
     double D = (a * c) - (b * b);  // always >= 0
     double sc;
     double tc;
   
     // compute the line parameters of the two closest points
     if (D < SMALL_NUM) {  // the lines are almost parallel
       sc = 0.0;
       tc = (b > c ? d / b : e / c);  // use the largest denominator
     } else {
       sc = (b * e - c * d) / D;
       tc = (a * e - b * d) / D;
     }
   
     // get the difference of the two closest points
     Eigen::Vector3d d_p = w + (sc * u) - (tc * v);  // =  L1(sc) - L2(tc)
   
     return d_p.norm();  // return the closest distance
   }
   
   inline double calc_distance_segment_to_segment3D(const Eigen::Ref<const Eigen::Vector3d>& line1_start, Eigen::Ref<const Eigen::Vector3d>& line1_end,
                                                    const Eigen::Ref<const Eigen::Vector3d>& line2_start, Eigen::Ref<const Eigen::Vector3d>& line2_end) {
     Eigen::Vector3d u = line1_end - line1_start;
     Eigen::Vector3d v = line2_end - line2_start;
     Eigen::Vector3d w = line2_start - line1_start;
     double a = u.squaredNorm();  // dot(u,u) always >= 0
     double b = u.dot(v);
     double c = v.squaredNorm();  // dot(v,v) always >= 0
     double d = u.dot(w);
     double e = v.dot(w);
     double D = (a * c) - (b * b);  // always >= 0
     double sc;
     double sN;
     double sD = D;  // sc = sN / sD, default sD = D >= 0
     double tc;
     double tN;
     double tD = D;  // tc = tN / tD, default tD = D >= 0
   
     // compute the line parameters of the two closest points
     if (D < SMALL_NUM) {  // the lines are almost parallel
       sN = 0.0;           // force using point P0 on segment S1
       sD = 1.0;           // to prevent possible division by 0.0 later
       tN = e;
       tD = c;
     } else {  // get the closest points on the infinite lines
       sN = (b * e - c * d);
       tN = (a * e - b * d);
       if (sN < 0.0) {  // sc < 0 => the s=0 edge is visible
         sN = 0.0;
         tN = e;
         tD = c;
       } else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
         sN = sD;
         tN = e + b;
         tD = c;
       }
     }
   
     if (tN < 0.0) {  // tc < 0 => the t=0 edge is visible
       tN = 0.0;
       // recompute sc for this edge
       if (-d < 0.0)
         sN = 0.0;
       else if (-d > a)
         sN = sD;
       else {
         sN = -d;
         sD = a;
       }
     } else if (tN > tD) {  // tc > 1  => the t=1 edge is visible
       tN = tD;
       // recompute sc for this edge
       if ((-d + b) < 0.0)
         sN = 0;
       else if ((-d + b) > a)
         sN = sD;
       else {
         sN = (-d + b);
         sD = a;
       }
     }
     // finally do the division to get sc and tc
     sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
     tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);
   
     // get the difference of the two closest points
     Eigen::Vector3d d_p = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)
   
     return d_p.norm();  // return the closest distance
   }
   
   template <typename VectorType>
   double calc_closest_point_to_approach_time(const VectorType& x1, const VectorType& vel1, const VectorType& x2, const VectorType& vel2) {
     VectorType dv = vel1 - vel2;
   
     double dv2 = dv.squaredNorm();  // dot(v,v)
     if (dv2 < SMALL_NUM)            // the  tracks are almost parallel
     {
       return 0.0;  // any time is ok.  Use time 0.
     }
   
     VectorType w0 = x1 - x2;
     double cpatime = -w0.dot(dv) / dv2;
   
     return cpatime;  // time of CPA
   }
   
   template <typename VectorType>
   double calc_closest_point_to_approach_distance(const VectorType& x1, const VectorType& vel1, const VectorType& x2, const VectorType& vel2, double bound_cpa_time = 0) {
     double ctime = calc_closest_point_to_approach_time<VectorType>(x1, vel1, x2, vel2);
     if (bound_cpa_time != 0 && ctime > bound_cpa_time) {
       ctime = bound_cpa_time;
     }
     VectorType p1 = x1 + (ctime * vel1);
     VectorType p2 = x2 + (ctime * vel2);
   
     return (p2 - p1).norm();  // distance at CPA
   }
   
   // dist_Point_to_Line(): get the distance of a point to a line
   //     Input:  a Point P and a Line L (in any dimension)
   //     Return: the shortest distance from P to L
   template <typename VectorType>
   double calc_distance_point_to_line(const VectorType& point, const VectorType& line_base, const VectorType& line_dir) {
     VectorType w = point - line_base;
   
     double c1 = w.dot(line_dir);
     double c2 = line_dir.dot(line_dir);
     double b = c1 / c2;
   
     VectorType pb = line_base + (b * line_dir);
     return (point - pb).norm();
   }
   //===================================================================
   
   // dist_Point_to_Segment(): get the distance of a point to a segment
   //     Input:  a Point P and a Segment S (in any dimension)
   //     Return: the shortest distance from P to S
   template <typename VectorType>
   double calc_distance_point_to_segment(const VectorType& point, const VectorType& line_start, const VectorType& line_end) {
     VectorType v = line_end - line_start;
     VectorType w = point - line_start;
   
     double c1 = w.dot(v);
     if (c1 <= 0) {
       return w.norm();
     }
   
     double c2 = v.dot(v);
     if (c2 <= c1) {
       return (point - line_end).norm();
     }
   
     double b = c1 / c2;
     VectorType pb = line_start + (b * v);
     return (point - pb).norm();
   }
   
   }  // namespace hateb_local_planner
   
   #endif /* DISTANCE_CALCULATIONS_H */
