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
#include "DelaunayBasedCorrection.h"

using namespace std;

void fillBottomFlatTriangle(cv::Mat& map_x, cv::Mat& map_y, cv::Mat homography, cv::Point pnt1, cv::Point pnt2, cv::Point pnt3)
{
	float invslope1 = (pnt2.x - pnt1.x) / (float)(pnt2.y - pnt1.y);
	float invslope2 = (pnt3.x - pnt1.x) / (float)(pnt3.y - pnt1.y);

	float curx1 = (float)pnt1.x;
	float curx2 = (float)pnt1.x;

	for (int y = pnt1.y; y <= pnt2.y; y++)
	{
		if (y < 0 || y >= map_x.size().height)
			continue;
		for (int x = (int)min(curx1, curx2); x <= (int)ceil(max(curx1, curx2)); x++) {
			if (x < 0 || x >= map_x.size().width)
				continue;
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
		if (y < 0 || y >= map_x.size().height)
			continue;
		for (int x = (int)min(curx1, curx2); x <= (int)ceil(max(curx1, curx2)); x++) {
			if (x < 0 || x >= map_x.size().width)
				continue;
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

void DelaunayBasedCorrection::findMaps(std::vector<cv::Point> cameraPoints, std::vector<cv::Point> projectionPoints, cv::Mat& map1, cv::Mat& map2, cv::Size projectionSize, cv::Size cameraSize, cv::Mat initialHomography, const int refinmentCount, const int distLimit) {
	DelaunayTriangulation triangulation(cameraPoints, projectionPoints, cameraSize);

	cv::Mat map_x(projectionSize, CV_32F, -1.f);
	cv::Mat map_y(projectionSize, CV_32F, -1.f);

	for (int i = refinmentCount; (--i) > 0;)
		triangulation.refineMesh(distLimit);
	vector<int> v = triangulation.getValidTriangles();
	
	//Iterate over all finite faces, drawing each triangle into the pixel map
	for (vector<int>::iterator v_it = v.begin(); v_it!=v.end();)
	{
		vector<int> ids({ *(v_it++), *(v_it++), *(v_it++) });
		//Sorting points by Y value
		if (projectionPoints[ids[0]].y > projectionPoints[ids[1]].y)
			swap(ids[0], ids[1]);
		if (projectionPoints[ids[0]].y > projectionPoints[ids[2]].y)
			swap(ids[0], ids[2]);
		if (projectionPoints[ids[1]].y > projectionPoints[ids[2]].y)
			swap(ids[1], ids[2]);

		const cv::Point p1 = projectionPoints[ids[0]], p2 = projectionPoints[ids[1]], p3 = projectionPoints[ids[2]];
		const vector<cv::Point2f> src({ p1, p2, p3 });
		const vector<cv::Point2f> dst({ cameraPoints[ids[0]], cameraPoints[ids[1]], cameraPoints[ids[2]] });

		//Getting homography from initial homography and current triangles affine transformation
		cv::Mat currentAffine = cv::Mat::eye(cv::Size(3, 3), CV_64F);
		cv::getAffineTransform(src, dst).copyTo(currentAffine(cv::Rect_<int>(0, 0, 3, 2)));
		const cv::Mat current_homography = initialHomography * currentAffine;
		if (current_homography.empty())
			continue;

		//Drawing the triangle split into a bottom-flat and top-flat triangles into the maps
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
	cv::convertMaps(map_x, map_y, map1, map2, CV_16SC2, false);
}