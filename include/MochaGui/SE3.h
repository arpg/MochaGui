#ifndef SE3_H
#define SE3_H

namespace Eigen {
    typedef Matrix<double,3,1> Vector3d ;
    typedef Matrix<double,4,1> Vector4d ;
    typedef Matrix<double,5,1> Vector5d ;
    typedef Matrix<double,6,1> Vector6d ;
    typedef Matrix<double,7,1> Vector7d ;
    typedef Matrix<double,3,3> Matrix3d ;
    typedef Matrix<double,4,4> Matrix4d ;

    typedef Matrix<double,6,6> Matrix6d ;

}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d Tinv(const Eigen::Matrix4d& T)
{
    Eigen::Matrix4d res = T;
    res.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
    res.block<3,1>(0,3) = -(T.block<3,3>(0,0).transpose())*T.block<3,1>(0,3);
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Tinv_2D(const Eigen::Matrix3d& T)
{
    Eigen::Matrix3d res = T;
    res.block<2,2>(0,0) = T.block<2,2>(0,0).transpose();
    res.block<2,1>(0,2) = -(T.block<2,2>(0,0).transpose())*T.block<2,1>(0,2);
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Cart2T_2D(const Eigen::Vector3d& cart)
{
    Eigen::Matrix3d res;
    res << cos(cart[2]), -sin(cart[2]), cart[0],
           sin(cart[2]),  cos(cart[2]), cart[1],
           0           ,  0           ,       1;
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d T2Cart_2D(const Eigen::Matrix3d& T)
{
    Eigen::Vector3d res;
    res[0] = T(0,2);
    res[1] = T(1,2);
    res[2] = atan2(T(1,0),T(0,0));
    return res;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d Cart2T(
        double x,
        double y,
        double z,
        double r,
        double p,
        double q
        )
{
    Eigen::Matrix4d T;
    // psi = roll, th = pitch, phi = yaw
    double cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    T(0,0) = cp*cq;
    T(0,1) = -cr*sq+sr*sp*cq;
    T(0,2) = sr*sq+cr*sp*cq;

    T(1,0) = cp*sq;
    T(1,1) = cr*cq+sr*sp*sq;
    T(1,2) = -sr*cq+cr*sp*sq;

    T(2,0) = -sp;
    T(2,1) = sr*cp;
    T(2,2) = cr*cp;

    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    T.row(3) = Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
    return T;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix4d Cart2T( const Eigen::Vector6d& C )
{
    double x = C(0);
    double y = C(1);
    double z = C(2);
    double r = C(3);
    double p = C(4);
    double q = C(5);

    return Cart2T(x, y, z, r, p, q);
}

////////////////////////////////////////////////////////////////////////////


inline Eigen::Vector3d R2Cart(const Eigen::Matrix3d& R) {
    Eigen::Vector3d rpq;
    //  roll
    rpq[0] = atan2(R(2, 1), R(2, 2));
    //  pitch
    double det = -R(2, 0) * R(2, 0) + 1.0;
    if (det <= 0) {
        if (R(2, 0) > 0) {
            rpq[1] = -M_PI / 2.0;
        } else {
            rpq[1] = M_PI / 2.0;
        }
    } else {
        rpq[1] = -asin(R(2, 0));
    }
    //  yaw
    rpq[2] = atan2(R(1, 0), R(0, 0));
    return rpq;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector6d T2Cart(const Eigen::Matrix4d& T) {
    Eigen::Vector6d Cart;
    Eigen::Vector3d rpq = R2Cart(T.block<3, 3>(0, 0));
    Cart(0) = T(0, 3);
    Cart(1) = T(1, 3);
    Cart(2) = T(2, 3);
    Cart(3) = rpq(0);
    Cart(4) = rpq(1);
    Cart(5) = rpq(2);
    return Cart;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Cart2R(
        const double& r,
        const double& p,
        const double& q
        )
{
    Eigen::Matrix3d R;
    // psi = roll, th = pitch, phi = yaw
    double cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    R(0,0) = cp*cq;
    R(0,1) = -cr*sq+sr*sp*cq;
    R(0,2) = sr*sq+cr*sp*cq;

    R(1,0) = cp*sq;
    R(1,1) = cr*cq+sr*sp*sq;
    R(1,2) = -sr*cq+cr*sp*sq;

    R(2,0) = -sp;
    R(2,1) = sr*cp;
    R(2,2) = cr*cp;
    return R;
}

////////////////////////////////////////////////////////////////////////////
inline Eigen::Matrix3d Cart2R(const Eigen::Vector3d& rpy) {
    return Cart2R(rpy(0), rpy(1), rpy(2));
}

#endif
