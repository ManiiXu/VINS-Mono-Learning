#include "solve_5pts.h"


namespace cv {
    void decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t )
    {

        Mat E = _E.getMat().reshape(1, 3);
        CV_Assert(E.cols == 3 && E.rows == 3);

        Mat D, U, Vt;
        SVD::compute(E, D, U, Vt);

        if (determinant(U) < 0) U *= -1.;
        if (determinant(Vt) < 0) Vt *= -1.;

        Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        W.convertTo(W, E.type());

        Mat R1, R2, t;
        R1 = U * W * Vt;
        R2 = U * W.t() * Vt;
        t = U.col(2) * 1.0;

        R1.copyTo(_R1);
        R2.copyTo(_R2);
        t.copyTo(_t);
    }

    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                         OutputArray _R, OutputArray _t, InputOutputArray _mask)
    {

        Mat points1, points2, cameraMatrix;
        _points1.getMat().convertTo(points1, CV_64F);
        _points2.getMat().convertTo(points2, CV_64F);
        _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

        int npoints = points1.checkVector(2);
        CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
                                  points1.type() == points2.type());

        CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

        if (points1.channels() > 1)
        {
            points1 = points1.reshape(1, npoints);
            points2 = points2.reshape(1, npoints);
        }

        double fx = cameraMatrix.at<double>(0,0);
        double fy = cameraMatrix.at<double>(1,1);
        double cx = cameraMatrix.at<double>(0,2);
        double cy = cameraMatrix.at<double>(1,2);

        points1.col(0) = (points1.col(0) - cx) / fx;
        points2.col(0) = (points2.col(0) - cx) / fx;
        points1.col(1) = (points1.col(1) - cy) / fy;
        points2.col(1) = (points2.col(1) - cy) / fy;

        points1 = points1.t();
        points2 = points2.t();

        Mat R1, R2, t;
        decomposeEssentialMat(E, R1, R2, t);
        Mat P0 = Mat::eye(3, 4, R1.type());
        Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
        P1(Range::all(), Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
        P2(Range::all(), Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
        P3(Range::all(), Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
        P4(Range::all(), Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

        // Do the cheirality check.
        // Notice here a threshold dist is used to filter
        // out far away points (i.e. infinite points) since
        // there depth may vary between postive and negtive.
        double dist = 50.0;
        Mat Q;
        triangulatePoints(P0, P1, points1, points2, Q);
        Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask1 = (Q.row(2) < dist) & mask1;
        Q = P1 * Q;
        mask1 = (Q.row(2) > 0) & mask1;
        mask1 = (Q.row(2) < dist) & mask1;

        triangulatePoints(P0, P2, points1, points2, Q);
        Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask2 = (Q.row(2) < dist) & mask2;
        Q = P2 * Q;
        mask2 = (Q.row(2) > 0) & mask2;
        mask2 = (Q.row(2) < dist) & mask2;

        triangulatePoints(P0, P3, points1, points2, Q);
        Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask3 = (Q.row(2) < dist) & mask3;
        Q = P3 * Q;
        mask3 = (Q.row(2) > 0) & mask3;
        mask3 = (Q.row(2) < dist) & mask3;

        triangulatePoints(P0, P4, points1, points2, Q);
        Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask4 = (Q.row(2) < dist) & mask4;
        Q = P4 * Q;
        mask4 = (Q.row(2) > 0) & mask4;
        mask4 = (Q.row(2) < dist) & mask4;

        mask1 = mask1.t();
        mask2 = mask2.t();
        mask3 = mask3.t();
        mask4 = mask4.t();

        // If _mask is given, then use it to filter outliers.
        if (!_mask.empty())
        {
            Mat mask = _mask.getMat();
            CV_Assert(mask.size() == mask1.size());
            bitwise_and(mask, mask1, mask1);
            bitwise_and(mask, mask2, mask2);
            bitwise_and(mask, mask3, mask3);
            bitwise_and(mask, mask4, mask4);
        }
        if (_mask.empty() && _mask.needed())
        {
            _mask.create(mask1.size(), CV_8U);
        }

        CV_Assert(_R.needed() && _t.needed());
        _R.create(3, 3, R1.type());
        _t.create(3, 1, t.type());

        int good1 = countNonZero(mask1);
        int good2 = countNonZero(mask2);
        int good3 = countNonZero(mask3);
        int good4 = countNonZero(mask4);

        if (good1 >= good2 && good1 >= good3 && good1 >= good4)
        {
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask1.copyTo(_mask);
            return good1;
        }
        else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
        {
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask2.copyTo(_mask);
            return good2;
        }
        else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
        {
            t = -t;
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask3.copyTo(_mask);
            return good3;
        }
        else
        {
            t = -t;
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask4.copyTo(_mask);
            return good4;
        }
    }

    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                         OutputArray _t, double focal, Point2d pp, InputOutputArray _mask)
    {
        Mat cameraMatrix = (Mat_<double>(3,3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);
        return cv::recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
    }
}

/**
 * @brief   通过求解本质矩阵得到R,t
 * @Description findFundamentalMat()采用RANSAC算法求解本质矩阵E
 *              recoverPose()通过本质矩阵得到Rt
                求Rt的对称变换，判断内点数大于12
 * @param[in]   corres  对应特征点对
 * @param[out]  Rotation    当前帧到参考帧的旋转矩阵
 * @param[out]  Translation 当前帧到参考帧的平移向量
 * @return      bool    true:内点数大于12
*/
bool MotionEstimator::solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &Rotation, Vector3d &Translation)
{
    if (corres.size() >= 15)
    {
        vector<cv::Point2f> ll, rr;
        for (int i = 0; i < int(corres.size()); i++)
        {
            ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
            rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
        }
        cv::Mat mask;
        
        /**
         *  Mat cv::findFundamentalMat(  返回通过RANSAC算法求解两幅图像之间的本质矩阵E
         *      nputArray  points1,             第一幅图像点的数组
         *      InputArray  points2,            第二幅图像点的数组
         *      int     method = FM_RANSAC,     RANSAC 算法
         *      double  param1 = 3.,            点到对极线的最大距离，超过这个值的点将被舍弃
         *      double  param2 = 0.99,          矩阵正确的可信度
         *      OutputArray mask = noArray()    在计算过程中没有被舍弃的点
         *  ) 
        */   
        cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 0.3 / 460, 0.99, mask);

        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Mat rot, trans;

        /**
         *  int cv::recoverPose (   通过本质矩阵得到Rt，返回通过手性校验的内点个数
         *      InputArray  E,              本质矩阵
         *      InputArray  points1,        第一幅图像点的数组
         *      InputArray  points2,        第二幅图像点的数组
         *      InputArray  cameraMatrix,   相机内参
         *      OutputArray     R,          第一帧坐标系到第二帧坐标系的旋转矩阵
         *      OutputArray     t,          第一帧坐标系到第二帧坐标系的平移向量
         *      InputOutputArray    mask = noArray()  在findFundamentalMat()中没有被舍弃的点
         *  )  
        */
        int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
        //cout << "inlier_cnt " << inlier_cnt << endl;

        //rr=R*ll+t     ll=(R^T)*rr-(R^T)*t
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        for (int i = 0; i < 3; i++)
        {   
            T(i) = trans.at<double>(i, 0);
            for (int j = 0; j < 3; j++)
                R(i, j) = rot.at<double>(i, j);
        }

        //得到窗口最后一帧（当前帧）到第l帧（参考帧）的坐标系变换Rt
        Rotation = R.transpose();
        Translation = -R.transpose() * T;
        if(inlier_cnt > 12)
            return true;
        else
            return false;
    }
    return false;
}



