/*
*   Single Point-of-view Projector Calibration System
*	Copyright (C) 2023 Damian Ziaber
*
*	This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*
*	This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License along with this program. If not, see https://www.gnu.org/licenses/.
*/
#include "pch.h"
#include "DelaunayBasedCorrection.h"

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, K>    Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                    Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds>                      Delaunay;
typedef Delaunay::Point                                             Point;
typedef Delaunay::Vertex_handle                                     Vertex_handle;

struct AutoIndex : public CGAL::cpp98::unary_function<const Point&, std::pair<Point, unsigned> > {
	mutable unsigned id;
	AutoIndex() : id(0) {}
	std::pair<Point, unsigned> operator()(const Point& pnt) const { return std::make_pair(pnt, id++); }
};

void fillBottomFlatTriangle(cv::Mat& map_x, cv::Mat& map_y, cv::Mat homography, cv::Point pnt1, cv::Point pnt2, cv::Point pnt3)
{
	float invslope1 = (pnt2.x - pnt1.x) / (float)(pnt2.y - pnt1.y);
	float invslope2 = (pnt3.x - pnt1.x) / (float)(pnt3.y - pnt1.y);

	float curx1 = (float)pnt1.x;
	float curx2 = (float)pnt1.x;

	for (int y = pnt1.y; y <= pnt2.y; y++)
	{
		for (int x = (int)min(curx1, curx2); x <= (int)ceil(max(curx1, curx2)); x++) {
			double div = homography.at<double>(2, 0) * (double)x + homography.at<double>(2, 1) * (double)y + homography.at<double>(2, 2);
			if (div == 0.0)
				div = 1e-20;
			map_x.at<float>(y, x) = (float)((homography.at<double>(0, 0) * (double)x + homography.at<double>(0, 1) * (double)y + homography.at<double>(0, 2)) / div);
			map_y.at<float>(y, x) = (float)((homography.at<double>(1, 0) * (double)x + homography.at<double>(1, 1) * (double)y + homography.at<double>(1, 2)) / div);
		}
		curx1 += invslope1;
		curx2 += invslope2;
	}
}

void fillTopFlatTriangle(cv::Mat& map_x, cv::Mat& map_y, cv::Mat homography, cv::Point pnt1, cv::Point pnt2, cv::Point pnt3)
{
	float invslope1 = (pnt3.x - pnt1.x) / (float)(pnt3.y - pnt1.y);
	float invslope2 = (pnt3.x - pnt2.x) / (float)(pnt3.y - pnt2.y);

	float curx1 = (float)pnt3.x;
	float curx2 = (float)pnt3.x;

	for (int y = pnt3.y; y >= pnt1.y; y--)
	{
		for (int x = (int)min(curx1, curx2); x <= (int)ceil(max(curx1, curx2)); x++) {
			double div = homography.at<double>(2, 0) * (double)x + homography.at<double>(2, 1) * (double)y + homography.at<double>(2, 2);
			if (div == 0.0)
				div = 1e-20;
			map_x.at<float>(y, x) = (float)((homography.at<double>(0, 0) * (double)x + homography.at<double>(0, 1) * (double)y + homography.at<double>(0, 2)) / div);
			map_y.at<float>(y, x) = (float)((homography.at<double>(1, 0) * (double)x + homography.at<double>(1, 1) * (double)y + homography.at<double>(1, 2)) / div);
		}
		curx1 -= invslope1;
		curx2 -= invslope2;
	}
}

inline bool invalidTriangle(cv::Point p1, cv::Point p2, cv::Point p3) {
	const int A = p2.x * p1.y + p3.x * p2.y + p1.x * p3.y;
	const int B = p1.x * p2.y + p2.x * p3.y + p3.x * p1.y;
	return B <= A + 5;
}

struct ErrorNode
{
	unsigned error;
	unsigned insert_time;
	unsigned id;

	bool operator<(const ErrorNode& o) const
	{
		return (error == o.error?insert_time > o.insert_time:error<o.error);
	}
};


void DelaunayBasedCorrection::findMaps(std::vector<cv::Point> cameraPoints, std::vector<cv::Point> projectionPoints, cv::Mat& map1, cv::Mat& map2, cv::Size projectionSize, cv::Mat initialHomography) {
	vector<Point> cameraPointsCGAL;
	cameraPointsCGAL.reserve(cameraPoints.size());
	for (auto pnt : cameraPoints)
		cameraPointsCGAL.push_back(Point(pnt.x, pnt.y));

	Delaunay delaunay;
	delaunay.insert(boost::make_transform_iterator(cameraPointsCGAL.begin(), AutoIndex()), boost::make_transform_iterator(cameraPointsCGAL.end(), AutoIndex()));

	//Saving vertex handles to array
	vector<Delaunay::Vertex_handle> handles(cameraPoints.size(), nullptr);
	for (Delaunay::Finite_vertices_iterator it = delaunay.finite_vertices_begin();
		it != delaunay.finite_vertices_end();
		it++)
	{
		const int id = it->info();
		handles[id] = it->handle();
	}
	unsigned insert_count = 0;

	// Initial error count calculation
	vector<unsigned> error_count(cameraPoints.size(), 0);
	priority_queue<ErrorNode> error_queue;
	for (Delaunay::Finite_faces_iterator it = delaunay.finite_faces_begin();
		it != delaunay.finite_faces_end();
		it++)
	{
		const vector<unsigned> ids({ it->vertex(0)->info(), it->vertex(1)->info(), it->vertex(2)->info() });
		if (invalidTriangle(projectionPoints[ids[0]], projectionPoints[ids[1]], projectionPoints[ids[2]])) {
			for (const unsigned id : ids)
				error_count[id]++;
		}
	}
	for (unsigned i = 0; i < cameraPoints.size(); i++)
		if(error_count[i] > 0)
			error_queue.push({ error_count[i], insert_count++, i });

	while (!error_queue.empty()) {
		const unsigned to_remove = error_queue.top().id;
		error_queue.pop();
		const Delaunay::Face_circulator start_circ = delaunay.incident_faces(handles[to_remove]);
		Delaunay::Face_circulator it = start_circ;
		do {
			if (!delaunay.is_infinite(it)) {
				const vector<unsigned> ids({ it->vertex(0)->info(), it->vertex(1)->info(), it->vertex(2)->info() });
				if (invalidTriangle(projectionPoints[ids[0]], projectionPoints[ids[1]], projectionPoints[ids[2]]))
					for (const unsigned id : ids) {
						error_count[id]--;
						if (error_count[id] != 0)
							error_queue.push({ error_count[id], insert_count++, id });
					}
			}
			it++;
		} while (it != start_circ);
		delaunay.remove(handles[to_remove]);

		vector<Delaunay::Face_handle> newFaces;
		delaunay.get_conflicts(cameraPointsCGAL[to_remove], std::back_inserter(newFaces));
		for (Delaunay::Face_handle face : newFaces)
			if (!delaunay.is_infinite(face)) {
				const vector<unsigned> ids({ face->vertex(0)->info(), face->vertex(1)->info(), face->vertex(2)->info() });
				if (invalidTriangle(projectionPoints[ids[0]], projectionPoints[ids[1]], projectionPoints[ids[2]]))
					for (const unsigned id : ids) {
						error_count[id]++;
						error_queue.push({ error_count[id], insert_count++, id });
					}
			}
		while (!error_queue.empty() && error_count[error_queue.top().id] != error_queue.top().error)
			error_queue.pop();
	}

	cv::Mat map_x(projectionSize, CV_32F, -1.f);
	cv::Mat map_y(projectionSize, CV_32F, -1.f);

	for (Delaunay::Finite_faces_iterator it = delaunay.finite_faces_begin();
		it != delaunay.finite_faces_end();
		it++)
	{
		//Sorting points by Y value
		vector<unsigned> ids({ it->vertex(0)->info(), it->vertex(1)->info(), it->vertex(2)->info() });
		if (projectionPoints[ids[0]].y > projectionPoints[ids[1]].y)
			swap(ids[0], ids[1]);
		if (projectionPoints[ids[0]].y > projectionPoints[ids[2]].y)
			swap(ids[0], ids[2]);
		if (projectionPoints[ids[1]].y > projectionPoints[ids[2]].y)
			swap(ids[1], ids[2]);

		const cv::Point p1 = projectionPoints[ids[0]], p2 = projectionPoints[ids[1]], p3 = projectionPoints[ids[2]];
		const vector<cv::Point2f> src({ p1, p2, p3 });
		const vector<cv::Point2f> dst({ cameraPoints[ids[0]], cameraPoints[ids[1]], cameraPoints[ids[2]] });

		//Getting homography
		cv::Mat currentAffine = cv::Mat::eye(cv::Size(3, 3), CV_64F);
		cv::getAffineTransform(src, dst).copyTo(currentAffine(cv::Rect_<int>(0, 0, 3, 2)));
		const cv::Mat current_homography = initialHomography * currentAffine;

		//Drawing triangles with given homography into map
		if (p2.y == p3.y)
			fillBottomFlatTriangle(map_x, map_y, current_homography, p1, p2, p3);
		else if (p1.y == p2.y)
			fillTopFlatTriangle(map_x, map_y, current_homography, p1, p2, p3);
		else {
			const cv::Point p4((int)(p1.x + ((float)(p2.y - p1.y) / (float)(p3.y - p1.y)) * (p3.x - p1.x)), p2.y);
			fillBottomFlatTriangle(map_x, map_y, current_homography, p1, p2, p4);
			fillTopFlatTriangle(map_x, map_y, current_homography, p2, p4, p3);
		}
	}
	//TODO: look into ways to smooth out maps or triangulation
	cv::convertMaps(map_x, map_y, map1, map2, CV_16SC2, false);
}