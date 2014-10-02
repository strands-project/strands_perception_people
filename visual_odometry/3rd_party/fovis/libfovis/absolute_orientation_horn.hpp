// Albert Huang <albert@csail.mit.edu>
//
// Implementation of 
//
//    Berthold K. P. Horn, 
//    "Closed-form solution of absolute orientation using unit quaternions",
//    Journal of the Optical society of America A, Vol. 4, April 1987

#ifndef __absolute_orientation_horn__
#define __absolute_orientation_horn__

#include <assert.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

/**
 * absolute_orientation_horn:
 * @P1: a matrix of dimension [3 x num_points]
 * @P2: a matrix of dimension [3 x num_points]
 * @result: output parameter
 *
 * Given two point sets P1 and P2, with each point p1_i in set P1 matched with
 * a point p2_i in P2, compute the rigid body transformation (isometry) M that
 * minimizes
 *
 * \sum ||p2_i - M p1_i||
 *
 * returns: 0 on success, -1 on failure
 */
template <typename DerivedA, typename DerivedB>
int absolute_orientation_horn(const Eigen::MatrixBase<DerivedA>& P1, 
        const Eigen::MatrixBase<DerivedB>& P2,
        Eigen::Isometry3d* result)
{
    int num_points = P1.cols();
    assert(P1.cols() == P2.cols());
    assert(P1.rows() == 3 && P2.rows() == 3);
    if(num_points < 3)
        return -1;

    // compute centroids of point sets
    Eigen::Vector3d P1_centroid = P1.rowwise().sum() / num_points;
    Eigen::Vector3d P2_centroid = P2.rowwise().sum() / num_points;

    Eigen::MatrixXd R1 = P1;
    R1.colwise() -= P1_centroid;
    Eigen::MatrixXd R2 = P2;
    R2.colwise() -= P2_centroid;

    // compute matrix M
    double Sxx = R1.row(0).dot(R2.row(0));
    double Sxy = R1.row(0).dot(R2.row(1));
    double Sxz = R1.row(0).dot(R2.row(2));
    double Syx = R1.row(1).dot(R2.row(0));
    double Syy = R1.row(1).dot(R2.row(1));
    double Syz = R1.row(1).dot(R2.row(2));
    double Szx = R1.row(2).dot(R2.row(0));
    double Szy = R1.row(2).dot(R2.row(1));
    double Szz = R1.row(2).dot(R2.row(2));

    double A00 = Sxx + Syy + Szz;
    double A01 = Syz - Szy;
    double A02 = Szx - Sxz;
    double A03 = Sxy - Syx;
    double A11 = Sxx - Syy - Szz;
    double A12 = Sxy + Syx;
    double A13 = Szx + Sxz;
    double A22 = -Sxx + Syy - Szz;
    double A23 = Syz + Szy;
    double A33 = -Sxx - Syy + Szz;

    // prepare matrix for eigen analysis
    Eigen::Matrix4d N;
    N << A00, A01, A02, A03,
        A01, A11, A12, A13,
        A02, A12, A22, A23,
        A03, A13, A23, A33;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(N);

    // rotation quaternion is the eigenvector with greatest eigenvalue
    Eigen::Vector4d eigvals = eigensolver.eigenvalues();
    int max_eigen_ind = 0;
    double max_eigen_val = eigvals(0);
    for(int i=1; i<4; i++) {
        if(eigvals(i) > max_eigen_val) {
            max_eigen_val = eigvals(i);
            max_eigen_ind = i;
        }
    }
    Eigen::Vector4d quat = eigensolver.eigenvectors().col(max_eigen_ind);
    Eigen::Quaterniond rotation(quat[0], quat[1], quat[2], quat[3]);
 
    // now compute the resulting isometry
    result->setIdentity();
    result->translate(P2_centroid - rotation * P1_centroid);
    result->rotate(rotation);

    return 0;
}
#endif
