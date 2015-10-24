//#pragma once
#include "Kalman.h"
#include "HungarianAlg.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class CTrack
{
public:
	vector<Point2d> trace;
	static size_t NextTrackID;
	size_t track_id;
	size_t skipped_frames; 
	Point2d prediction;
	TKalmanFilter* KF;
	CTrack(Point2f p, float dt, float Accel_noise_mag);
	~CTrack();
};


class CTracker
{
public:
	
	// Ўаг времени опроса фильтра
    // Step time for the filter
	float dt; 

	float Accel_noise_mag;

	// ѕорог рассто€ни€. ≈сли точки наход€тс€ дуг от друга на рассто€нии,
    // превышающем этот порог, то эта пара не рассматриваетс€ в задаче о назначени€х.
    // The threshold distance. If the points are separated by a distance
    // greater than this threshold, then this pair is not considered the problem of assignments
	double dist_thres;
	// ћаксимальное количество кадров которое трек сохран€етс€ не получа€ данных о измерений.
    // Maximum number of frames that the track saves to not receive data measurements
	int maximum_allowed_skipped_frames;
	// ћаксимальна€ длина следа
    // Maximum length of the track
	int max_trace_length;

	vector<CTrack*> tracks;
	void Update(vector<Point2d>& detections);
	CTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=10,int _max_trace_length=10);
	~CTracker(void);
};

