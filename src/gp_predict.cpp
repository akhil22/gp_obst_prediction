
//gaussian process functions 
#include<gaussian_process_catkin/SingleGP.h>
#include<gaussian_process_catkin/covarianceFunctions.h>
#include<gp_obst_prediction/gp_predict_functions.h>

#include<iostream>
#include<fstream>
#include<sstream>
#include<dirent.h>

using namespace ros;

int main(int argc,char** argv){ 
//	cout<<RunTest("wc -l /home/akhil/traj/center/pt1.txt")<<endl;
	gp_obst_prediction::GpPredictFunctions gp_predict;
	int num_points;
	
	TVector<TDoubleVector> data = gp_predict.ReadTrajectory("/home/akhil/traj/center/pt1.txt",&num_points);                                                     
	TVector<TDoubleVector> inputs(num_points);
	TVector<double> outputs(num_points);
        outputs = gp_predict.GetTrainingData(data,&inputs,2,0,num_points);
//	for (int i=0;i<num_points;i++){
//		cout<<"inputs:"<<inputs(i)(0)<<" "<<"outputs: "<<outputs(i)<<endl;
//	}
	CovFuncND cov(1,0.5381,0.1282);
	gaussian_process::SingleGP gp(cov,0.1000);
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
