#include<iostream>
#include<C:\Users\Henyue Guan\Desktop\Algorithm\eigen-3.4.0\Eigen\Dense>

class KalmanFilter
{
private:
	int stateSize;  // size of the state
	int meaSize;   // size of teh measurement
	int controlSize;  // size of the control input

	Eigen::MatrixXd A;  // state matrix
	Eigen::VectorXd x;  // state vector
	Eigen::MatrixXd B;  // control matrix
	Eigen::VectorXd u;  // control vector
	Eigen::MatrixXd C;  // observation matrix
	Eigen::VectorXd y;  // measurement vector

	Eigen::MatrixXd P;  // covariance of the state
	Eigen::MatrixXd Q;  // covariance of the dynamic error
	Eigen::MatrixXd R;  // covariance of the measurement error

public:
	KalmanFilter(int stateSize, int meaSize, int controlSize)
	{
		this->stateSize = stateSize;
		this->meaSize = meaSize;
		this->controlSize = controlSize;

		x.resize(stateSize);
		A.resize(stateSize, stateSize);
		B.resize(stateSize, controlSize);
		u.resize(controlSize);
		P.resize(stateSize, stateSize);
		Q.resize(stateSize, stateSize);
		R.resize(stateSize, stateSize);
	}

	void MatrixInit(Eigen::MatrixXd& A, Eigen::MatrixXd& B, 
		Eigen::MatrixXd& C, Eigen::MatrixXd& R, Eigen::MatrixXd& Q)
	{
		this->A = A;
		this->B = B;
		this->C = C;
		this->R = R;
		this->Q = Q;
	}

	Eigen::VectorXd predict(Eigen::VectorXd& x_Posterior, Eigen::MatrixXd& P_Posterior)
	{
		// input the initialization at first use
		// input the posterior state and covariancde
		this->x = x_Posterior;
		this->P = P_Posterior;
		x = A * x + B * u;  // get the priori state
		P = A * P * A.transpose() + Q;  // get the priori covariance
		y = C * x;  // get the measurement

		return x;
	}

	Eigen::VectorXd update()
	{
		// calculate the kalman gain, get teh posterior state and covariance
		Eigen::MatrixXd temp = (C * P * C.transpose() + R).inverse();
		Eigen::MatrixXd K = P * C.transpose() * temp;  // get the kalman gain
		x = x + K * (y - C * x);  // get the posterior state
		Eigen::MatrixXd I;
		I.setIdentity();
		P = (I-K * C) * P;  // get the poseterior state
	}
};
