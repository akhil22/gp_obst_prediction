#include<iostream>
#include<fstream>
#include<sstream>
#include<dirent.h>
#include<string>
#include<vector>
#include<stdlib.h>
#include<ros/ros.h>
#include<ros/console.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<dynamic_reconfigure/server.h>
using namespace std;

//structure representing trajectory data
struct Trajectory{
	string file_name;
	int num_points;
	TVector<TDoubleVector> inputs;
	TDoubleVector time;
	TDoubleVector x;
	TDoubleVector y;
	TDoubleVector vx;
	TDoubleVector vy;
	TDoubleVector hyper_param_x;
	TDoubleVector hyper_param_y;
};

class GpObstaclePrediction{
	public:
		GpObstaclePrediction();
	private:
		string command_;
		int num_traj_;                              
		string dataset_dir_;
		string hyper_param_file_;
		vector<int> input_indexes_;
		ros::NodeHandle nh_;
		
		// holds trajectory information for all the trajectories
		Trajectory* trajectories_; 
	        
		//simple shell commands executer
		string runCommand( const string &command);

		//get all the trajectories
		void getAllTrajectories();

		//read single trajectory from file
		void readTrajectory(int traj_number);

		//set the hyperparameters for all trajectories
		void setHyperParams();

		//calculate velocities data from (x,t) and (y,t) for all the trajectories
		void calVelocities();
};

int main(int argc,char** argv){
	ros::init(argc,argv,"gp_obst_precition");
	GpObstaclePrediction gp_obts_prediction;
	return 0;
}

GpObstaclePrediction::GpObstaclePrediction(){

	//get all the parameters from ros patameter server
	ros::NodeHandle private_nh_("~");
	private_nh_.param("dataset_dir",dataset_dir_,string("~/traj"));
	private_nh_.param("input_indexes",input_indexes_,vector<int>(1,0));
	private_nh_.param("hyper_param_file",hyper_param_file_,string("~/hyp.txt"));
	
	//get total num of trajectories in a dataset
	string temp_output = runCommand(string("ls -l ")+dataset_dir_+string(" | wc -l"));
	sscanf(temp_output.c_str(),"%d",&num_traj_);
	num_traj_ = num_traj_ - 1;
	
	//initialize trajectories
	trajectories_ = new Trajectory [num_traj_];
	
	//get all trajectories calculate velocities and set hyperparameters
	getAllTrajectories();
	calVelocities();
	setHyperParams();
	
}

void GpObstaclePrediction::calVelocities(){
	for(int j=0; j<num_traj_; j++){
		for(int i = 0;i< trajectories_[j].num_points-1; i++){
			double tempvx = (trajectories_[j].x(i+1) - trajectories_[j].x(i))/(trajectories_[j].time(i+1)-trajectories_[j].time(i));
			double tempvy = (trajectories_[j].y(i+1) - trajectories_[j].y(i))/(trajectories_[j].time(i+1)-trajectories_[j].time(i));
			trajectories_[j].vx.insert_element(i,tempvx);
			trajectories_[j].vy.insert_element(i,tempvy);
		}
	}
}

void GpObstaclePrediction::setHyperParams(){
	ifstream filep(hyper_param_file_.c_str());
	string line;
	int j=0;
	while(getline(filep,line)){
		double temp_data[6];
		stringstream ss2;
		ss2<<line;
		string value;
		int i = 0;
		while(getline(ss2,value,',')){
			sscanf(value.c_str(),"%lf",&temp_data[i]);
			i++;
		}
		for(int k=0;k<6;k++){
			if(!(k/3))
		        	trajectories_[j].hyper_param_x.insert_element(k,temp_data[k]);
			else
		        	trajectories_[j].hyper_param_y.insert_element(k-3,temp_data[k]);
		}
		j++;
	}
}

void GpObstaclePrediction::getAllTrajectories(){
	string temp_output = runCommand(string("ls ")+dataset_dir_);
	stringstream ss(temp_output);
	for(int i=0; i<num_traj_; i++){
		char temp_file_name[100];
		ss>>temp_file_name;
		trajectories_[i].file_name = temp_file_name;
		readTrajectory(i);
	}

}

string GpObstaclePrediction::runCommand(const string &Command)
{
	FILE *fd = NULL;
	char Buffer[ BUFSIZ ];
	size_t pos = string::npos;
	string InputLine = "";
	fd = popen( Command.data(), "r" );
	stringstream ss;
	if(NULL != fd)
	{
		while(fgets( Buffer, sizeof( Buffer ), fd ) )
		{
			InputLine = Buffer;

			ss <<InputLine;

		}	/*	while( fgets() )	*/

		pclose( fd );

		fd = NULL;

	}	/*	if( NULL != fd )	*/
	else
	{
		cerr << "Error opening " << Command << endl;

	}	/*	if( NULL == fd )	*/
	return ss.str();
}


void GpObstaclePrediction::readTrajectory(int traj_number){
	
	//get number of points
	stringstream ss;
	ss<<"wc -l "<<dataset_dir_<<"/"<<trajectories_[traj_number].file_name;
	string temp_op = runCommand(ss.str());
	sscanf(temp_op.c_str(),"%d",&trajectories_[traj_number].num_points);
	
	//initilize all the vectors
	trajectories_[traj_number].x.resize(trajectories_[traj_number].num_points);
	trajectories_[traj_number].y.resize(trajectories_[traj_number].num_points);
	trajectories_[traj_number].time.resize(trajectories_[traj_number].num_points);
	trajectories_[traj_number].vx.resize(trajectories_[traj_number].num_points-1);
	trajectories_[traj_number].vy.resize(trajectories_[traj_number].num_points-1);
	trajectories_[traj_number].inputs.resize(trajectories_[traj_number].num_points);
	trajectories_[traj_number].hyper_param_x.resize(6);
	trajectories_[traj_number].hyper_param_y.resize(6);
	
	//read the trajectory file
	ifstream filep((dataset_dir_+string("/")+string(trajectories_[traj_number].file_name)).c_str());
	string line;
	int j=0;
	while(getline(filep,line)){
		double temp_data[3];
		stringstream ss2;
		ss2<<line;
		string value;
		int i = 0;
		while(getline(ss2,value,',')){
			sscanf(value.c_str(),"%lf",&temp_data[i]);
			i++;
		}

		//construct a vector to form inputs 
		TDoubleVector td(input_indexes_.size());
		for(int k=0; k<input_indexes_.size(); k++){
			td(k) = temp_data[input_indexes_[k]];
		}
		
		trajectories_[traj_number].x.insert_element(j,temp_data[0]);
		trajectories_[traj_number].y.insert_element(j,temp_data[1]);
		trajectories_[traj_number].time.insert_element(j,temp_data[2]);
		trajectories_[traj_number].inputs.insert_element(j,td);
		j++;
	}

}
