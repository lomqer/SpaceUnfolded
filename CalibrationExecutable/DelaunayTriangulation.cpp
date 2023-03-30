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

#include "DelaunayTriangulation.h"

#include <iostream>

using namespace std;

bool inCircumcircle(const cv::Point& tri1, const cv::Point& tri2, const cv::Point& tri3, const cv::Point& d) {
	const int a1 = tri1.x - d.x;
	const int a2 = tri1.y - d.y;
	const int b1 = tri2.x - d.x;
	const int b2 = tri2.y - d.y;
	const int c1 = tri3.x - d.x;
	const int c2 = tri3.y - d.y;

	const long long a3 = a1 * a1 + a2 * a2;
	const long long b3 = b1 * b1 + b2 * b2;
	const long long c3 = c1 * c1 + c2 * c2;

	return a1 * b2 * c3 + a2 * b3 * c1 + a3 * b1 * c2 < (a3* b2* c1 + a1 * b3 * c2 + a2 * b1 * c3);
}

inline int getSide(const cv::Point& e1, const cv::Point& e2, const cv::Point& p) {
	return (e1.x - p.x) * (e2.y - p.y) - (e1.y - p.y) * (e2.x - p.x);
}

DelaunayTriangulation::Edge* DelaunayTriangulation::createEdge(int from, int to) {
	Edge* e = (Edge*)malloc(sizeof(Edge));
	Edge* e_sym = (Edge*)malloc(sizeof(Edge));
	e->origin = from; e->destination = to; e->ccw = e; e->cw = e; e->valid = true;
	e_sym->origin = to; e_sym->destination = from; e_sym->ccw = e_sym; e_sym->cw = e_sym; e_sym->valid = true;
	e->sym = e_sym;
	e_sym->sym = e;
	allEdges.push_back(e);
	allEdges.push_back(e_sym);
	leadingVertexEdges[from] = e;
	leadingVertexEdges[to] = e_sym;
	return e;
}

void DelaunayTriangulation::deleteEdge(Edge* e) {
	e->valid = false; e->sym->valid = false;
	if (e->cw->valid)
		leadingVertexEdges[e->origin] = e->cw;
	else
		leadingVertexEdges[e->origin] = nullptr;
	if (e->sym->cw->valid)
		leadingVertexEdges[e->destination] = e->sym->cw;
	else
		leadingVertexEdges[e->destination] = nullptr;
	spliceEdges(e, e->cw);
	spliceEdges(e->sym, e->sym->cw);
}

void DelaunayTriangulation::spliceEdges(Edge* e1, Edge* e2) {
	if (e1!=e2) {
		swap(e1->ccw->cw, e2->ccw->cw);
		swap(e1->ccw, e2->ccw);
	}
}

DelaunayTriangulation::Edge* DelaunayTriangulation::connectEdges(Edge* e1, Edge* e2) {
	Edge* newEdge = createEdge(e1->destination, e2->origin);
	spliceEdges(newEdge, e1->sym->cw);
	spliceEdges(newEdge->sym, e2);
	return newEdge;
}

inline cv::Point DelaunayTriangulation::getPoint(const int& index) {
	return points[pointIndices[index]];
}

inline cv::Point DelaunayTriangulation::getProjectionPoint(const int& index) {
	return projectionPoints[pointIndices[index]];
}

void DelaunayTriangulation::triangulate(int start, int end, Edge** leftMostOut, Edge** rightMostOut) {
	if (end == start + 2) {
		Edge* e = createEdge(start, start+1);
		*leftMostOut = e;
		*rightMostOut = e->sym;
		return;
	}
	else if (end == start + 3) {
		Edge* e1 = createEdge(start, start + 1);
		Edge* e2 = createEdge(start + 1, start + 2);
		spliceEdges(e1->sym, e2);

		const int side = getSide(getPoint(e1->origin), getPoint(e1->destination), getPoint(start+2));
		if (side > 0) {
			connectEdges(e2, e1);
			*leftMostOut = e1;
			*rightMostOut = e2->sym;
			return;
		}
		else if (side < 0) {
			Edge* e3 = connectEdges(e2, e1);
			*leftMostOut = e3->sym;
			*rightMostOut = e3;
			return;
		}
		*leftMostOut = e1;
		*rightMostOut = e2->sym;
		return;
	}
	else {
		const int mid = (start + end + 1) / 2;

		Edge* insideLeft, * outsideLeft;
		triangulate(start, mid, &outsideLeft, &insideLeft);

		Edge* insideRight, * outsideRight;
		triangulate(mid, end, &insideRight, &outsideRight);

		while (true) {
			if (getSide(getPoint(insideLeft->origin), getPoint(insideLeft->destination), getPoint(insideRight->origin)) > 0)
				insideLeft = insideLeft->sym->ccw;
			else if (getSide(getPoint(insideRight->origin), getPoint(insideRight->destination), getPoint(insideLeft->origin)) < 0)
				insideRight = insideRight->sym->cw;
			else
				break;
		}

		Edge* baseEdge = connectEdges(insideLeft->sym, insideRight);
		if (insideLeft->origin == outsideLeft->origin)
			outsideLeft = baseEdge;
		if (insideRight->origin == outsideRight->origin)
			outsideRight = baseEdge->sym;

		Edge* rc = baseEdge->sym->ccw, * lc = baseEdge->cw;
		bool rrc = getSide(getPoint(baseEdge->origin), getPoint(baseEdge->destination), getPoint(rc->destination)) > 0;
		bool rlc = getSide(getPoint(baseEdge->origin), getPoint(baseEdge->destination), getPoint(lc->destination)) > 0;
		while (rrc || rlc) {
			if (rrc) {
				while (getSide(getPoint(baseEdge->origin), getPoint(baseEdge->destination), getPoint(rc->ccw->destination))
					&& inCircumcircle(getPoint(baseEdge->destination), getPoint(baseEdge->origin), getPoint(rc->destination), getPoint(rc->ccw->destination))) {
					Edge* temp = rc->ccw;
					deleteEdge(rc);
					rc = temp;
				}
			}
			if (rlc) {
				while (getSide(getPoint(baseEdge->origin), getPoint(baseEdge->destination), getPoint(lc->cw->destination))
					&& inCircumcircle(getPoint(baseEdge->destination), getPoint(baseEdge->origin), getPoint(lc->destination), getPoint(lc->cw->destination))) {
					Edge* temp = lc->cw;
					deleteEdge(lc);
					lc = temp;
				}
			}
			if (!rrc or (rlc && inCircumcircle(getPoint(rc->destination), getPoint(rc->origin), getPoint(lc->origin), getPoint(lc->destination)))) {
				baseEdge = connectEdges(lc, baseEdge->sym);
			}
			else {
				baseEdge = connectEdges(baseEdge->sym, rc->sym);
			}
			rc = baseEdge->sym->ccw;
			lc = baseEdge->cw;
			rrc = getSide(getPoint(baseEdge->origin), getPoint(baseEdge->destination), getPoint(rc->destination)) > 0;
			rlc = getSide(getPoint(baseEdge->origin), getPoint(baseEdge->destination), getPoint(lc->destination)) > 0;
		}
		*leftMostOut = outsideLeft;
		*rightMostOut = outsideRight;
	}
}

void DelaunayTriangulation::cleanupEdges() {
	vector<Edge*> removedEdges;
	for (Edge* e : allEdges)
		if (!(e->valid))
			removedEdges.push_back(e);
	allEdges.erase(std::remove_if(allEdges.begin(), allEdges.end(), [=](const Edge* e) {return !(e->valid);}), allEdges.end());
	for (Edge* e : removedEdges)
		std::free(e);
}

inline long long squaredDistance(const cv::Point& a, const cv::Point& b) {
	return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

long long DelaunayTriangulation::powerOfPoint(cv::Point pnt, cv::Point tri1, cv::Point tri2, cv::Point tri3) {
	const long long tri12 = squaredDistance(tri1, tri2);
	const long long tri13 = squaredDistance(tri1, tri3);
	const long long tri23 = squaredDistance(tri2, tri3);
	const long long tri1pnt = squaredDistance(tri1, pnt);
	const long long tri2pnt = squaredDistance(tri2, pnt);
	const long long tri3pnt = squaredDistance(tri3, pnt);

	const long long p = (tri12 + tri13 + tri23) * (tri12 + tri1pnt + tri2pnt) * (tri13 + tri1pnt + tri3pnt) * (tri23 + tri2pnt + tri3pnt);
	const long long q = 2 * tri12 * tri13 * tri23 * tri1pnt * tri2pnt * tri3pnt;
	return p - q;
}

long long DelaunayTriangulation::powerOfPoint(cv::Point pnt, DelaunayTriangulation::Edge* edge) {
	if (getSide(getPoint(edge->cw->destination), getPoint(edge->ccw->destination), getPoint(edge->origin)) > 0 || getSide(getPoint(edge->cw->destination), getPoint(edge->ccw->destination), getPoint(edge->destination)) <= 0)
		return numeric_limits<long long>::max();
	return powerOfPoint(pnt, getPoint(edge->cw->destination), getPoint(edge->destination), getPoint(edge->ccw->destination));
}


struct PointPowerEdgePair {
	PointPowerEdgePair(double pointPower, DelaunayTriangulation::Edge* edge) :pointPower(pointPower), edge(edge) {};

	double pointPower;
	DelaunayTriangulation::Edge* edge = nullptr;
	boost::heap::fibonacci_heap< struct PointPowerEdgePair>::handle_type ccw;
	boost::heap::fibonacci_heap< struct PointPowerEdgePair>::handle_type cw;
	bool operator <(const struct PointPowerEdgePair& other) const {
		return pointPower > other.pointPower;
	}
} typedef PointPowerEdgePair;


vector<DelaunayTriangulation::Edge*> DelaunayTriangulation::removeVertex(int id) {
	typedef boost::heap::fibonacci_heap< PointPowerEdgePair > fibonacci_heap;
	typedef fibonacci_heap::handle_type heap_handle;
	const cv::Point removedPoint = getPoint(id);

	fibonacci_heap heap;
	Edge* start_edge = leadingVertexEdges[id];
	Edge* current_edge = start_edge->ccw;
	heap_handle start_handle = heap.push(PointPowerEdgePair(powerOfPoint(removedPoint, start_edge), start_edge));
	heap_handle last_handle = start_handle;
	
	while (start_edge != current_edge) {
		heap_handle h = heap.push(PointPowerEdgePair(powerOfPoint(removedPoint, current_edge), current_edge));
		(*last_handle).ccw = h;
		(*h).cw = last_handle;
		last_handle = h;
		current_edge = current_edge->ccw;
	}
	(*last_handle).ccw = start_handle;
	(*start_handle).cw = last_handle;
	vector<Edge*> newEdges;
	if(heap.size()>2)
		newEdges.reserve(heap.size() - 3);
	while (heap.size()>3) {
		const cv::Point destinationPoint = getPoint(heap.top().edge->destination);
		deleteEdge(heap.top().edge);
		heap_handle ccw = heap.top().ccw;
		heap_handle cw = heap.top().cw;
		heap.pop();
		if (getSide(getPoint((*cw).edge->destination), getPoint((*ccw).edge->destination), destinationPoint) > 0)
			newEdges.push_back(connectEdges((*cw).edge, (*ccw).edge->sym));
		(*cw).pointPower = powerOfPoint(removedPoint, (*cw).edge);
		heap.update(cw);
		(*ccw).pointPower = powerOfPoint(removedPoint, (*ccw).edge);
		heap.update(ccw);
		(*cw).ccw = ccw;
		(*ccw).cw = cw;
	}
	while (!heap.empty()) {
		deleteEdge(heap.top().edge);
		heap.pop();
	}
	return newEdges;
}

struct HeapNode {
	const unsigned id;
	const unsigned init;
	unsigned err;

	HeapNode(const unsigned node_id, const unsigned init, unsigned error) : id(node_id), init(init), err(error) { }
	bool operator <(const struct HeapNode& other) const {
		return err < other.err || (err == other.err && init > other.init);
	}
};

bool invalidTriangle(cv::Point p1, cv::Point p2, cv::Point p3) {
	const int A = p2.x * p1.y + p3.x * p2.y + p1.x * p3.y;
	const int B = p1.x * p2.y + p2.x * p3.y + p3.x * p1.y;
	return A <= B;
}

void DelaunayTriangulation::removeWrongTriangles() {
	typedef boost::heap::fibonacci_heap< HeapNode > fibonacci_heap;
	typedef fibonacci_heap::handle_type heap_handle;
	heap_handle* heap_handles = (heap_handle*)malloc(points.size() * sizeof(heap_handle));
	vector<bool> in_heap(points.size(), false);
	fibonacci_heap error_heap;
	unsigned init = 0;
	for (Edge* leadingEdge : leadingVertexEdges) {
		Edge* edge = leadingEdge;
		if (edge==nullptr || pointIndices[edge->origin] >= projectionPoints.size())
			continue;
		unsigned errorCount = 0;
		do {
			Edge* nextEdge = edge->ccw;
			if (pointIndices[edge->destination] < projectionPoints.size()
				&& pointIndices[nextEdge->destination] < projectionPoints.size()
				&& invalidTriangle(getProjectionPoint(edge->destination), getProjectionPoint(nextEdge->destination), getProjectionPoint(edge->origin))) {
				errorCount++;
			}
			edge = nextEdge;
		} while (edge != leadingEdge);
		if (errorCount != 0) {
			const int i = edge->origin;
			in_heap[i] = true;
			heap_handles[i] = error_heap.push(HeapNode(i, init++, errorCount));
		}
	}
	while (!error_heap.empty() && error_heap.top().err != 0) {
		const unsigned to_remove = error_heap.top().id;
		error_heap.pop();

		//Go through all triangles which will be destroyed when we remove the point with most errors,
		//and adjust the affected points error counts
		Edge* startEdge = leadingVertexEdges[to_remove];
		Edge* edge = startEdge;
		do {
			Edge* nextEdge = edge->ccw;
			if (pointIndices[edge->destination] < projectionPoints.size()
				&& pointIndices[nextEdge->destination] < projectionPoints.size()
				&& invalidTriangle(getProjectionPoint(edge->destination), getProjectionPoint(nextEdge->destination), getProjectionPoint(edge->origin))) {
				(*(heap_handles[nextEdge->destination])).err--;
				error_heap.update(heap_handles[nextEdge->destination]);
				(*(heap_handles[edge->destination])).err--;
				error_heap.update(heap_handles[edge->destination]);
			}
			edge = nextEdge;
		} while (edge != startEdge);

		vector<Edge*> newEdges = removeVertex(to_remove);
		for (Edge* e : newEdges) {
			e->valid = false;
			e->sym->valid = false;
		}
		for (Edge* e : newEdges) {
			e->valid = true;
			e->sym->valid = true;
			for (const Edge* e2 : { e, e->sym }) {
				if (e2->ccw->sym->valid && e2->sym->cw->valid
					&& pointIndices[e2->origin] < projectionPoints.size()
					&& pointIndices[e2->destination] < projectionPoints.size()
					&& pointIndices[e2->ccw->destination] < projectionPoints.size()
					&& invalidTriangle(getProjectionPoint(e2->destination), getProjectionPoint(e2->ccw->destination), getProjectionPoint(e2->origin))) {
					for (const int& pnt : { e2->origin, e2->destination, e2->ccw->destination }) {
						if (!in_heap[pnt]) {
							in_heap[pnt] = true;
							heap_handles[pnt] = error_heap.push(HeapNode(pnt, init++, 1));
						} else {
							(*(heap_handles[pnt])).err++;
							error_heap.update(heap_handles[pnt]);
						}
					}
				}
			}
		}
	}
	std::free(heap_handles);
}

void DelaunayTriangulation::closeCorners() {
	for (const int id : {0, 1, indicesSize-2, indicesSize-1}) {
		typedef boost::heap::fibonacci_heap< PointPowerEdgePair > fibonacci_heap;
		typedef fibonacci_heap::handle_type heap_handle;
		const cv::Point removedPoint = getPoint(id);

		fibonacci_heap heap;
		Edge* start_edge = leadingVertexEdges[id];
		Edge* current_edge = start_edge->ccw;
		heap_handle start_handle = heap.push(PointPowerEdgePair(powerOfPoint(removedPoint, start_edge), start_edge));
		heap_handle last_handle = start_handle;

		while (start_edge != current_edge) {
			heap_handle h = heap.push(PointPowerEdgePair(powerOfPoint(removedPoint, current_edge), current_edge));
			(*last_handle).ccw = h;
			(*h).cw = last_handle;
			last_handle = h;
			current_edge = current_edge->ccw;
		}
		(*last_handle).ccw = start_handle;
		(*start_handle).cw = last_handle;
		while (heap.size() > 3) {
			if (pointIndices[heap.top().edge->destination] >= projectionPoints.size()) {
				(*heap.top().ccw).cw = heap.top().cw;
				(*heap.top().cw).ccw = heap.top().ccw;
				deleteEdge(heap.top().edge);
				heap.pop();
				continue;
			}

			Edge* cwEdge = heap.top().edge->cw;
			Edge* ccwEdge = heap.top().edge->ccw;

			const cv::Point destinationPoint = getPoint(heap.top().edge->destination);
			const cv::Point destinationProjectionPoint = getProjectionPoint(heap.top().edge->destination);

			heap_handle ccw = heap.top().ccw;
			heap_handle cw = heap.top().cw;

			const bool valid = (getSide(getPoint(heap.top().edge->origin), destinationPoint, getPoint(cwEdge->destination)) > 0) != (getSide(getPoint(heap.top().edge->origin), destinationPoint, getPoint(ccwEdge->destination)) > 0);
			
			deleteEdge(heap.top().edge);
			heap.pop();
			
			if (valid && pointIndices[cwEdge->destination] < projectionPoints.size() && pointIndices[ccwEdge->destination] < projectionPoints.size() && getSide(getPoint(cwEdge->destination), getPoint(ccwEdge->destination), destinationPoint) > 0 && !invalidTriangle(getProjectionPoint(cwEdge->destination), destinationProjectionPoint, getProjectionPoint(ccwEdge->destination)))
				connectEdges(cwEdge, ccwEdge->sym);
			
			(*cw).pointPower = powerOfPoint(removedPoint, (*cw).edge);
			heap.update(cw);
			
			(*ccw).pointPower = powerOfPoint(removedPoint, (*ccw).edge);
			heap.update(ccw);
			
			(*cw).ccw = ccw;
			(*ccw).cw = cw;
		}
		while (!heap.empty()) {
			deleteEdge(heap.top().edge);
			heap.pop();
		}
	}
}

vector<int> DelaunayTriangulation::getValidTriangles() {
	closeCorners();
	cleanupEdges();
	vector<int> triangles;
	triangles.reserve(allEdges.size());
	for (Edge* leadingEdge : leadingVertexEdges) {
		Edge* edge = leadingEdge;
		if (edge != nullptr)
			do {
				if (edge->valid && edge->ccw->sym->valid && edge->sym->cw->valid) {
					edge->valid = false; edge->ccw->sym->valid = false; edge->sym->cw->valid = false;
					triangles.push_back(pointIndices[edge->destination]);
					triangles.push_back(pointIndices[edge->ccw->destination]);
					triangles.push_back(pointIndices[edge->origin]);
				}
				edge = edge->ccw;
			} while (edge != leadingEdge);
	}
	for (Edge* e : allEdges) {
		e->valid = true;
	}
	return triangles;
}

void DelaunayTriangulation::refineMesh(int distLimit) {
	const int distLimitSquared = distLimit * distLimit;
	for (Edge* leadingEdge : leadingVertexEdges) {
		Edge* edge = leadingEdge;
		if (edge != nullptr && pointIndices[edge->origin] < projectionPoints.size()) {
			vector<cv::Point2f> neighboursCamera;
			vector<cv::Point2f> neighboursProjector;
			do {
				if (pointIndices[edge->destination] < projectionPoints.size()) {
					neighboursCamera.push_back(getPoint(edge->destination));
					neighboursProjector.push_back(getProjectionPoint(edge->destination));
				}
				edge = edge->ccw;
			} while (edge != leadingEdge);

			if (neighboursCamera.size() >= 3) {
				const cv::Point currectCameraPoint = getPoint(edge->origin);
				const cv::Point currectProjectionPoint = getProjectionPoint(edge->origin);
				cv::Point guessPoint;
				if (neighboursCamera.size() == 3) {
					const cv::Mat affine = cv::getAffineTransform(neighboursCamera, neighboursProjector);
					if (affine.empty())
						continue;
					const int x = (int)round((affine.at<double>(0, 0) * (double)currectCameraPoint.x + affine.at<double>(0, 1) * (double)currectCameraPoint.y + affine.at<double>(0, 2)));
					const int y = (int)round((affine.at<double>(1, 0) * (double)currectCameraPoint.x + affine.at<double>(1, 1) * (double)currectCameraPoint.y + affine.at<double>(1, 2)));
					guessPoint = cv::Point(x, y);
				}
				else {
					const cv::Mat homography = cv::findHomography(neighboursCamera, neighboursProjector, cv::noArray());
					if (homography.empty())
						continue;
					double div = homography.at<double>(2, 0) * (double)currectCameraPoint.x + homography.at<double>(2, 1) * (double)currectCameraPoint.y + homography.at<double>(2, 2);
					if (div == 0.0)
						div = 1e-20;
					const int x = (int)round((homography.at<double>(0, 0) * (double)currectCameraPoint.x + homography.at<double>(0, 1) * (double)currectCameraPoint.y + homography.at<double>(0, 2)) / div);
					const int y = (int)round((homography.at<double>(1, 0) * (double)currectCameraPoint.x + homography.at<double>(1, 1) * (double)currectCameraPoint.y + homography.at<double>(1, 2)) / div);
					guessPoint = cv::Point(x, y);
				}
				const int dist = (guessPoint.x - currectProjectionPoint.x) * (guessPoint.x - currectProjectionPoint.x) + (guessPoint.y - currectProjectionPoint.y) * (guessPoint.y - currectProjectionPoint.y);
				if (dist < distLimitSquared)
					projectionPoints[pointIndices[edge->origin]] = (guessPoint * (distLimitSquared - dist) + projectionPoints[pointIndices[edge->origin]] * dist) / distLimitSquared;
			}
		}
	}
	removeWrongTriangles();
}

DelaunayTriangulation::~DelaunayTriangulation() {
	free(pointIndices);
}

DelaunayTriangulation::DelaunayTriangulation(vector<cv::Point>& points, vector<cv::Point>& projectionPoints, cv::Size cameraSize) : points(points), projectionPoints(projectionPoints) {
	this->points.push_back(cv::Point(-1, -1));
	this->points.push_back(cv::Point(cameraSize.width, cameraSize.height));
	this->points.push_back(cv::Point(cameraSize.width, -1));
	this->points.push_back(cv::Point(-1, cameraSize.height));
	this->pointIndices = (int*)malloc(this->points.size() * sizeof(int));
	iota(pointIndices, pointIndices + this->points.size(), 0);
	std::sort(pointIndices, pointIndices + this->points.size(), [this](unsigned current, unsigned other) {
		return this->points[current].x < this->points[other].x || (this->points[current].x == this->points[other].x && this->points[current].y > this->points[other].y);
		});
	//Remove duplicates
	for (int* i = pointIndices + 1; i != pointIndices + this->points.size(); i++)
		if (this->points[*(i - 1)].x == this->points[*(i)].x && this->points[*(i - 1)].y == this->points[*(i)].y)
			*(i - 1) = -1;
	int* indicesEnd = std::remove_if(pointIndices, pointIndices + this->points.size(), [=](const int& x) {return x == -1; });
	this->indicesSize = indicesEnd - pointIndices;
	this->leadingVertexEdges = vector<Edge*>(indicesSize, nullptr);

	Edge *left, *right;
	triangulate(0, indicesEnd - pointIndices, &left, &right);
	removeWrongTriangles();
}