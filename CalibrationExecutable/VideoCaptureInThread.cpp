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

#include "VideoCaptureInThread.h"

VideoCaptureInThread::VideoCaptureInThread(int cameraId) {
	this->cap = cv::VideoCapture(cameraId, cv::CAP_DSHOW);
	this->capturingThread = std::thread([this]() {
		while (this->cap.isOpened()) {
			cv::Mat frame;
			this->cap.read(frame);
			this->mtx.lock();
			this->lastFrame = frame;
			this->mtx.unlock();
		}
		});
}

bool VideoCaptureInThread::isOpened() {
	return this->cap.isOpened();
}

cv::Mat VideoCaptureInThread::read() {
	this->mtx.lock();
	cv::Mat to_return;
	lastFrame.copyTo(to_return);
	this->mtx.unlock();
	return to_return;
}

void VideoCaptureInThread::close() {
	this->cap.release();
	this->capturingThread.join();
}