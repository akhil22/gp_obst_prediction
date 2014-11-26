#include<gaussian_process_catkin/covarianceFunctions.h>
#include<ros/ros.h>
#include<stdlib.h>
using namespace std;

//runs system commands and get the outputs
namespace gp_obst_prediction{
	class GpPredictFunctions{
		public:
			void GpPredcitFunctions();
			
			string RunTest( const string &Command);

			//read a trajectory from a text file
			TVector<TDoubleVector> ReadTrajectory(const char* file_name,int* num_points);

			//get the training data for a particular trajectory
			TVector<double>GetTrainingData(TVector<TDoubleVector> data, TVector<TDoubleVector> *inputs,int input_field,int output_field, int num_points);
	};
}
