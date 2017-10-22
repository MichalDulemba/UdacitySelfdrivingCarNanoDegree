#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO: DONE
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO: DONE
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
//recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //TODO: YOUR CODE HERE
    float px2py2 = px*px + py*py;
    float sqrtpx2py2 = sqrt(px2py2);
    float thirdpowersqrtpx2py2 = sqrtpx2py2 * sqrtpx2py2 * sqrtpx2py2;

      if (px2py2 == 0){
      Hj    <<  0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 0;
      }
      else
      {
    Hj    <<   px/sqrtpx2py2, py/sqrtpx2py2, 0, 0,
           -py/px2py2, px/px2py2, 0, 0,
           py*(vx*py-vy*px)/thirdpowersqrtpx2py2, px*(vy*px-vx*py)/thirdpowersqrtpx2py2, px/sqrtpx2py2, py/sqrtpx2py2;
      }

    //check division by zero

    //compute the Jacobian matrix

    return Hj;


}
