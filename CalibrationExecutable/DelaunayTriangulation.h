/*
*   Single Point-of-view Projector Calibration System
*	Copyright (C) 2023 Damian Ziaber
*
*	This program is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*
*	This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
*
*	You should have received a copy of the GNU Lesser General Public License along with this program. If not, see https://www.gnu.org/licenses/.
*/

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <numeric>
#include <limits>
#include <boost/heap/fibonacci_heap.hpp>

#pragma once
class DelaunayTriangulation
{
public:
	DelaunayTriangulation(std::vector<cv::Point>& points, std::vector<cv::Point>& projectionPoints, cv::Size cameraSize);
	~DelaunayTriangulation();
	std::vector<int> getValidTriangles();
	void refineMesh(int distLimit);

	struct Edge {
		int origin;
		int destination;
		struct Edge* sym;
		struct Edge* ccw;
		struct Edge* cw;
		bool valid;
	} typedef Edge;

private:
	long long powerOfPoint(cv::Point pnt, DelaunayTriangulation::Edge* e);
	long long powerOfPoint(cv::Point pnt, cv::Point tri1, cv::Point tri2, cv::Point tri3);
	
	std::vector<cv::Point>& points;
	std::vector<cv::Point>& projectionPoints;
	int* pointIndices;
	int indicesSize;
	std::vector<Edge*> allEdges;
	std::vector<Edge*> leadingVertexEdges;
	
	Edge* createEdge(int from, int to);
	void deleteEdge(Edge* e);
	DelaunayTriangulation::Edge* connectEdges(Edge* e1, Edge* e2);
	void spliceEdges(Edge* e1, Edge* e2);
	std::vector<Edge*> removeVertex(int id);

	cv::Point getPoint(const int& index);
	cv::Point getProjectionPoint(const int& index);

	void triangulate(int start, int end, Edge** leftMost, Edge** rightMost);
	void cleanupEdges();
	void closeCorners();
	void removeWrongTriangles();
};

