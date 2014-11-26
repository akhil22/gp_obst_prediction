#include<gaussian_process_catkin/SingleGP.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<ros/ros.h>
#include<stdlib.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<dirent.h>
using namespace std;
using namespace ros;
using namespace gaussian_process;
string RunTest( const string &Command)
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
TVector<TDoubleVector> ReadTrajectory(const char* file_name,int* num_points){
	stringstream ss;
	ss<<"wc -l "<<file_name;
	string temp_op = RunTest(ss.str());
	sscanf(temp_op.c_str(),"%d",num_points);
	cout<<*num_points<<endl;
	TVector<TDoubleVector> data(*num_points);
//	data = new TVector<TDoubleVector>[*num_points];
	ifstream filep(file_name);
	string line;
	int j=0;
	while(getline(filep,line)){
		TDoubleVector td(3);
		stringstream ss2;
		ss2<<line;
		string value;
		int i=0;
		while(getline(ss2,value,',')){
			float temp;
			sscanf(value.c_str(),"%f",&temp);
			td(i) = temp;
			i++;
		}
		data(j) = td;
		j++;
	}
	return data;

}
TVector<double>GetTrainingDate(TVector<TDoubleVector> data, TVector<TDoubleVector> *inputs,int input_field,int output_field, int num_points){
	TVector<double> outputs(num_points);
	TVector<TDoubleVector> temp_inputs(num_points);
//	inputs = new TVector<TDoubleVector> [1];
	for(int i=0;i<num_points;i++){
		TDoubleVector td(1);
		td(0) = data(i)(input_field);
	        temp_inputs(i) = td;
		outputs(i) = data(i)(output_field);
	}
	*inputs=temp_inputs;
	return outputs;
}
int main(int argc,char** argv){ 
//	cout<<RunTest("wc -l /home/akhil/traj/center/pt1.txt")<<endl;
	int num_points;
	TVector<TDoubleVector> data=ReadTrajectory("/home/akhil/traj/center/pt1.txt",&num_points);                                                     
	TVector<TDoubleVector> inputs(num_points);
	TVector<double> outputs(num_points);
        outputs = GetTrainingDate(data,&inputs,2,0,num_points);
//	for (int i=0;i<num_points;i++){
//		cout<<"inputs:"<<inputs(i)(0)<<" "<<"outputs: "<<outputs(i)<<endl;
//	}
	CovFuncND cov(1,0.5381,0.1282);
	SingleGP gp(cov,0.1000);
        gp.SetData(inputs,outputs);
	cout<<gp.GetDataLikelihood()<<endl;
	TDoubleVector testPoints[num_points-1];
	double target_mean[num_points-1];
	double targetVar[num_points-1];
	ofstream ofile;
	ofile.open("/home/akhil/traj/center/pt1_out.txt");
//	ofile <<"alkjh\n";
	for(int i=0;i<num_points-1;i++){
		testPoints[i] = (inputs(i)+inputs(i+1))/2.0;
	        gp.Evaluate(testPoints[i],target_mean[i],targetVar[i]);
		ofile <<target_mean[i]<<","<<testPoints[i](0)<<"\n";
	}
	ofile.close();

//	gp.OptimizeGP();
//	cout<<cov.evalParam[1]<<endl;
}
