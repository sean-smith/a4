#include "Ctracker.h"
using namespace cv;
using namespace std;

size_t CTrack::NextTrackID=0;
// ---------------------------------------------------------------------------
// Track constructor.
// The track begins from initial point (pt)
// ---------------------------------------------------------------------------
CTrack::CTrack(Point2f pt, float dt, float Accel_noise_mag)
{
	track_id=NextTrackID;

	NextTrackID++;
	// Every track have its own Kalman filter,
	// it user for next point position prediction.
	KF = new TKalmanFilter(pt,dt,Accel_noise_mag);
	// Here stored points coordinates, used for next position prediction.
	prediction=pt;
	skipped_frames=0;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTrack::~CTrack()
{
	// Free resources.
	delete KF;
}

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length)
{
dt=_dt;
Accel_noise_mag=_Accel_noise_mag;
dist_thres=_dist_thres;
maximum_allowed_skipped_frames=_maximum_allowed_skipped_frames;
max_trace_length=_max_trace_length;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(vector<Point2d>& detections)
{
	// -----------------------------------
	// If there is no tracks yet, then every point begins its own track.
	// -----------------------------------
	if(tracks.size()==0)
	{
		// If no tracks yet
		for(int i=0;i<detections.size();i++)
		{
			CTrack* tr=new CTrack(detections[i],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}

	// -----------------------------------
    // If it is tracking already anyway
	// -----------------------------------
    int N=tracks.size();		// Tracks
    int M=detections.size();	// How many detectors

    // Matrix of distances from the N-th track to the M-th detector
	vector< vector<double> > Cost(N,vector<double>(M));
    vector<int> assignment; // Destination

	// -----------------------------------
	// Треки уже есть, составим матрицу расстояний
    // From the tracks we already have, create the matrix of distances
	// -----------------------------------
	double dist;
	for(int i=0;i<tracks.size();i++)
	{	
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for(int j=0;j<detections.size();j++)
		{
			Point2d diff=(tracks[i]->prediction-detections[j]);
			dist=sqrtf(diff.x*diff.x+diff.y*diff.y);
			Cost[i][j]=dist;
		}
	}
	// -----------------------------------
    // Solving assignment problem (filter tracks and predictions)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

	// -----------------------------------
	// почистим assignment от пар с большим расстоянием
    // Will clean assignments of pairs with long distance
	// -----------------------------------
    // Unassigned tracks
	vector<int> not_assigned_tracks;

	for(int i=0;i<assignment.size();i++)
	{
		if(assignment[i]!=-1)
		{
			if(Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i]=-1;
                // Mark the unassigned tracks and increase the counter of dropped frames,
                // when the number of dropped frames exceed the threshold value, the track is erased
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{			
            // If the track is not assigned to a detector, then increment the missed shots.
			tracks[i]->skipped_frames++;
		}

	}

	// -----------------------------------
    // If the track does not get a detector for a long time, remove
	// -----------------------------------
	for(int i=0;i<tracks.size();i++)
	{
		if(tracks[i]->skipped_frames>maximum_allowed_skipped_frames)
		{
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
		}
	}
	// -----------------------------------
    // Identify unassigned detectors
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(int i=0;i<detections.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	// -----------------------------------
    // and start new tracks
	// -----------------------------------
	if(not_assigned_detections.size()!=0)
	{
		for(int i=0;i<not_assigned_detections.size();i++)
		{
			CTrack* tr=new CTrack(detections[not_assigned_detections[i]],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}

    // Update status of filters

	for(int i=0;i<assignment.size();i++)
	{
		// Если трек апдейтился меньше одного раза, то состояние фильтра некорректно.

		tracks[i]->KF->GetPrediction();

        if(assignment[i]!=-1) // If the assigment has the update on it
		{
			tracks[i]->skipped_frames=0;
			tracks[i]->prediction=tracks[i]->KF->Update(detections[assignment[i]],1);
        }else				  // If not, we continue to predict
		{
			tracks[i]->prediction=tracks[i]->KF->Update(Point2f(0,0),0);	
		}
		
		if(tracks[i]->trace.size()>max_trace_length)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(),tracks[i]->trace.end()-max_trace_length);
		}

		tracks[i]->trace.push_back(tracks[i]->prediction);
		tracks[i]->KF->LastResult=tracks[i]->prediction;
	}

}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void)
{
	for(int i=0;i<tracks.size();i++)
	{
	delete tracks[i];
	}
	tracks.clear();
}
