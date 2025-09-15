#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
using namespace Eigen;

class LQR
{

public:
  LQR();
  Matrix<float, 3, 2> computeB(const Vector3f &X_cur, float dt);
  Vector3f computeStateSpace(Matrix3f &A, Matrix<float, 3, 2> &B,
                             Vector3f &X_prev, Vector2f &u_prev, float dt);
  Vector2f computeInput(Matrix3f &A, Matrix<float, 3, 2> &B, Matrix3f &Q,
                        Matrix2f &R, Vector3f &X_ref, Vector3f &X_cur,
                        float dt);

private:
  Vector3f X_ref_;
  // float dt_;

  // Calculating them, storing as class members for future debug
  // Vector3f X_cur_;
  // Vector3f X_prev_;
  // Vector3f u_prev_;
  // Vector3f u_cur_;
};
