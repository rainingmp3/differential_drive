#include "lqr.hpp"
LQR::LQR() {};
Matrix<float, 3, 2> LQR::computeB(const Vector3f &X_cur, float dt)
{
  // Diff drive specific
  // B (3x2)
  Matrix<float, 3, 2> B;
  B << std::cos(X_cur(2)) * dt, 0, 
       std::sin(X_cur(2)) * dt, 0,
                             0, dt;
  return B;
};

Vector3f LQR::computeStateSpace(Matrix3f &A, Matrix<float, 3, 2> &B,
                                Vector3f &X_prev, Vector2f &u_prev, float dt)
{
  // X_cur = A * X_prev + B*u_prev;
  Vector3f X_cur_ = A * X_prev + B * u_prev;
  // TODO: Add clipping

  return X_cur_;
};

Vector2f LQR::computeInput(Matrix3f &A, Matrix<float, 3, 2> &B, Matrix3f &Q,
                           Matrix2f &R, Vector3f &X_ref, Vector3f &X_cur,
                           float dt)
{
  Vector3f X_err = X_ref - X_cur;
  const int N = 50; // Number of interations to get optimal control input
  Matrix3f P[N + 1];
  // We assume starting point of P weight to be Q, as they match in dimensions
  // and why not
  P[N] = Q;

  // for N...1 -> P[i-1] = f(p[i])
  for (int i = N; i > 0; --i)
  {
    P[i - 1] = Q + A.transpose() * P[i] * A -
               (A.transpose() * P[i] * B) *
                   (R + B.transpose() * P[i] * B)
                       .completeOrthogonalDecomposition()
                       .pseudoInverse() *
                   (B.transpose() * P[i] * A);
  }

  Matrix<float, 2, 3> K[N];
  Vector2f u[N];
  for (int i = 0; i < N; ++i)
  {
    // Calculate the optimal feedback gain K
    K[i] = -(R + B.transpose() * P[i + 1] * B)
                .completeOrthogonalDecomposition()
                .pseudoInverse() *
           B.transpose() * P[i + 1] * A;

    // Calculate the control input
    u[i] = - K[i] * X_err;
  }

  // Optimal control input is u_star
  return u[N - 1];
  // for i to N -> K[i]
};
