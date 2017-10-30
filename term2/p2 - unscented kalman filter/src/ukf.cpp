#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;


  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;


 // later try different values
 //00551 - almost

  x_  <<  0,
          0,
          0,
          0,
          1;
 // later try different values

  P_  <<  1,0,0,0,0,
          0,1,0,0,0,
          0,0,1,0,0,
          0,0,0,1,0,
          0,0,0,0,1;


VectorXd x_ = VectorXd(n_x_);

//create example covariance matrix
MatrixXd P_ = MatrixXd(n_x_, n_x_);



  lambda_= 3 - n_aug_;
  weights_ =  VectorXd::Zero(2*n_aug_ + 1);

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i< 2 * n_aug_ + 1; i++){
                 ////std:: cout << i;
                   weights_(i) = 1 / (2 * (lambda_ + n_aug_));
                       //std:: cout << weights(i);
          }
  //std::cout<< "\n" << "weights \n" << weights_ << "\n";


  MatrixXd Xsig_pred_ = MatrixXd::Zero(n_x_, 2*n_x_ + 1);
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

double UKF::NormAng(double angle){
   angle = atan2(sin(angle), cos(angle));
   if (angle < -M_PI) {
     angle = angle + 2*M_PI;

   }
   else if (angle > M_PI) {
     angle = angle - 2*M_PI;
   }
   return angle;
}
/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

    if (!is_initialized_) {


      if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */

        //          		meas_package.raw_measurements_ << ro,theta, ro_dot;
        // ro * cos (theta)
        //ro * sin (theta)
        //cout << "init radar"  << "\n";
        x_(0) = meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]);
        x_(1) = meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]);

      }
      else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        /**
        Initialize state.
        */

        //meas_package.raw_measurements_ << px, py;
        //cout << "init lidar" << "\n";
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
      }


      previous_t = meas_package.timestamp_;
      // need other way to initialize time

      // done initializing, no need to predict or update
      is_initialized_ = true;
      return;
      }

      float delta_t = (meas_package.timestamp_ - previous_t)/1000000.0;
      Prediction(delta_t);

      if (meas_package.sensor_type_ == MeasurementPackage::LASER){

        UpdateLidar(meas_package);
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){

        UpdateRadar(meas_package);
      }
      previous_t = meas_package.timestamp_;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //
  // FIRST STEP _ AUGMENTED SIGMA POINTS
  //
  /*
  x_ <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
                 -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
                  0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
                 -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
                 -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

   */

    // std :: cout << "\n------------------------------- PREDICTION --------------------------------------- \n\n\n";

  //create augmented mean vector
    VectorXd x_aug = VectorXd::Zero(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd::Zero(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

    //std::cout << "\n x \n " << x_ << "\n";
    //std::cout << "\n P \n " << P_ << "\n";

      //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //std::cout << "\n x_aug \n" << x_aug <<"\n";

    //create augmented covariance matrix
    P_aug = MatrixXd::Zero(7,7);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;
    //std::cout << "\n P_aug \n" << P_aug<< "\n\n";

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    //std::cout << "\n L \n" << L << "\n\n";
    // create coefficient
    double coef = sqrt(lambda_ +n_aug_);

    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;

    // std::cout << Xsig_aug.col(0) << "\n";

    for (int i = 0; i< n_aug_; i++)
    {
      Xsig_aug.col(i+1)       = x_aug + coef * L.col(i);
      Xsig_aug.col(i+1+n_aug_) = x_aug - coef * L.col(i);
    }
    //std:: cout << "PREDICTION xsig aug\n " << Xsig_aug << "\n\n";
    //std:: cout << "PREDICTION P_aug \n " << P_aug << "\n\n";


    //
    //     SECOND STEP PREDICT SIGMA POINTS
    //

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

     double px_old, py_old, v_old, psi_old, psi_dot_old, vi_ak, vi_psi_k;
      double px, py, v, psi, psi_dot;
      double px_noise, py_noise, v_noise, psi_noise, psi_dot_noise;

      for (int i=0; i<2*n_aug_+1; i++){
           px_old = Xsig_aug(0,i);
           py_old = Xsig_aug(1,i);
           v_old = Xsig_aug(2,i);
           psi_old = Xsig_aug(3,i);
           psi_dot_old = Xsig_aug(4,i);
           vi_ak = Xsig_aug(5,i);
           vi_psi_k = Xsig_aug(6,i);

      if (fabs(psi_dot_old) > 0.0001) {
         px = px_old + v_old/psi_dot_old *(sin(psi_old +psi_dot_old*delta_t)-sin(psi_old));
         py = py_old + v_old/psi_dot_old *(-cos(psi_old +psi_dot_old*delta_t)+cos(psi_old));
      }
      else {
         px = px_old + v_old *(cos(psi_old));
         py = py_old + v_old *(sin(psi_old));
      }
      v = v_old;
      psi = psi_old + psi_dot_old * delta_t;
      psi_dot = psi_dot_old;


      px_noise = delta_t*delta_t/2*cos(psi_old)*vi_ak;
      py_noise = delta_t*delta_t/2*sin(psi_old)*vi_ak;
      v_noise = delta_t * vi_ak;
      psi_noise = delta_t*delta_t/2 * vi_psi_k;
      psi_dot_noise = delta_t * vi_psi_k;

      Xsig_pred_(0,i) = px+px_noise;
      Xsig_pred_(1,i) = py+py_noise;
      Xsig_pred_(2,i) = v+v_noise;
      Xsig_pred_(3,i) = psi+psi_noise;
      Xsig_pred_(4,i) = psi_dot + psi_dot_noise;
    }
    //std:: cout << "PREDICTION xsig_pred \n " << Xsig_pred_ << "\n\n";

    //
    // THIRD STEP - PREDICT MEAN AND covariance
    //

    VectorXd mat = VectorXd(n_x_);

    //std::cout << "\n x \n " << x_ << "\n";
    //std::cout << "\n P \n " << P_ << "\n";

     x_.fill(0.0);
     for (int i=0; i<(2 * n_aug_ + 1); i++){
         //std:: cout << "i " << i;
         x_ = x_ + weights_(i) * Xsig_pred_.col(i);
     }


     //std::cout << "PREDICTION x_ \n" << x_  << "\n\n";

     P_.fill(0.0);
       for (int i=0; i<(2 * n_aug_ + 1); i++){
         //std:: cout << "i " << i;

         mat(0) = Xsig_pred_(0,i)-x_(0);
         mat(1) = Xsig_pred_(1,i)-x_(1);
         mat(2) = Xsig_pred_(2,i)-x_(2);
         mat(3) = Xsig_pred_(3,i)-x_(3);
         mat(4) = Xsig_pred_(4,i)-x_(4);

        //angle normalization - find better way

        mat(3) = NormAng(mat(3));

        //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;


         P_ = P_ + weights_(i)*mat*mat.transpose();

       //std::cout << "mat" << mat << "\n" << "i" << i << "\n";
       }
     //std::cout << "PREDICTION P_ \n" << P_  << "\n\n";


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //std :: cout << "\n------------------------------- LIDAR UPDATE  --------------------------------------- \n\n\n";

  MatrixXd H_ = MatrixXd(2, 5);
	H_ << 1, 0, 0, 0, 0,
			  0, 1, 0, 0, 0;

  double px = meas_package.raw_measurements_[0];
  double py = meas_package.raw_measurements_[1];
  VectorXd z = VectorXd(2);

  z << px, py;

  //cout << "z lidar \n" << z << "\n\n";

  MatrixXd R = MatrixXd::Zero(2,2);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;


  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

  //std::cout << "UPDATE LIDAR P_ \n" << P_  << "\n\n";
  //std::cout << "UPDATE LIDAR x_ \n" << x_  << "\n\n";
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //std :: cout << "\n------------------------------- RADAR UPDATE  --------------------------------------- \n\n\n";
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  int n_z = 3;
  MatrixXd Zsig = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd::Zero(n_z);
  MatrixXd S = MatrixXd::Zero(n_z,n_z);
  MatrixXd R = MatrixXd::Zero(3,3);
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  VectorXd z = VectorXd(n_z);
  double rho_in = meas_package.raw_measurements_[0];
  double phi_in = meas_package.raw_measurements_[1];
  double rho_dot_in = meas_package.raw_measurements_[2];

  z << rho_in, phi_in, rho_dot_in;
  //cout << "z radar \n" << z << "\n\n";

  double px,py,v,psi,ro,theta,ro_dot;

  // fix for the lidar
  //transform sigma points into measurement space
  for (int i=0; i<2 * n_aug_ + 1;i++){

  px = Xsig_pred_(0,i);
  py = Xsig_pred_(1,i);
  v = Xsig_pred_(2,i);
  psi = Xsig_pred_(3,i);

  ro = sqrt(px*px+py*py);
  theta =  atan2(py,px);
  ro_dot = ((px*cos(psi))*v+(py*sin(psi)*v)) / ro;
  Zsig(0,i) = ro;
  Zsig(1,i) = theta;
  Zsig(2,i) = ro_dot;
  }
  //std::cout << "w" << weights.rows() << "\n";
  //std::cout << "w" <<weights.cols() << "\n";

  //std::cout << "zsig" <<Zsig.rows() << "\n";
  //std::cout <<"zsig" <<Zsig.cols() << "\n";

  //std::cout <<"zpred" <<z_pred.rows() << "\n";
  //std::cout <<"zpred" <<z_pred.cols() << "\n";


  for (int i=0; i<2 * n_aug_ + 1;i++){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }



  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;

  VectorXd mat = VectorXd(n_x_);

  for (int i=0; i<2 * n_aug_ + 1;i++){

      // angle norm should be added
  mat = Zsig.col(i) - z_pred;
  S = S+ weights_(i) * mat * mat.transpose();
 //  S = weights(i) * (Zsig.col(i)- )(Zsig.col(i)).transpose();
    }


  S=S+R;


//update state

  MatrixXd part1 = MatrixXd();
  MatrixXd part2 = MatrixXd();

  for (int i=0; i<2 * n_aug_ + 1; i++){
      part1 = Xsig_pred_.col(i)-x_;

      //angle norm

      part2 = Zsig.col(i)-z_pred;
      Tc = Tc + weights_(i) * part1 * part2.transpose();


      //angle norm
  }
 MatrixXd K = Tc * S.inverse();
 x_ = x_ + K * (z-z_pred);
 P_ = P_ - K * S * K.transpose();

 //std::cout << "UPDATE RADAR P_ \n" << P_  << "\n\n";
 //std::cout << "UPDATE RADAR x_ \n" << x_  << "\n\n";

}
