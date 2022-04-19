#include <min_snap/min_snap_closeform.h>

    namespace my_planner
{
    minsnapCloseform::minsnapCloseform(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        n_order = 7;
        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        mean_vel = meanvel;
    }

    void minsnapCloseform::Init(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        n_order = 7;
        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        mean_vel = meanvel;
    }

    // 0 means each segment time is one second.
    void minsnapCloseform::init_ts(int init_type)
    {
        const double dist_min = 2.0;
        ts = Eigen::VectorXd::Zero(n_seg);
        if (init_type)
        {
            Eigen::VectorXd dist(n_seg);
            double dist_sum = 0, t_sum = 0;
            for (int i = 0; i < n_seg; i++)
            {
                dist(i) = 0;
                for (int j = 0; j < 3; j++)
                {
                    dist(i) += pow(wps[i + 1](j) - wps[i](j), 2);
                }
                dist(i) = sqrt(dist(i));
                if ((dist(i)) < dist_min)
                {
                    dist(i) = sqrt(dist(i)) * 2;
                }
                dist_sum += dist(i);
            }
            dist(0) += 1;
            dist(n_seg - 1) += 1;
            dist_sum += 2;
            double T = dist_sum / mean_vel;
            for (int i = 0; i < n_seg - 1; i++)
            {
                ts(i) = dist(i) / dist_sum * T;
                t_sum += ts(i);
            }
            ts(n_seg - 1) = T - t_sum;
        }
        else
        {
            for (int i = 0; i < n_seg; i++)
            {
                ts(i) = 1;
            }
        }
        cout << "ts: " << ts.transpose() << endl;
    }

    int minsnapCloseform::fact(int n)
    {
        if (n < 0)
        {
            cout << "ERROR fact(" << n << ")" << endl;
            return 0;
        }
        else if (n == 0)
        {
            return 1;
        }
        else
        {
            return n * fact(n - 1);
        }
    }

    Eigen::VectorXd minsnapCloseform::calDecVel(const Eigen::VectorXd decvel)
    {
        Eigen::VectorXd temp(Eigen::VectorXd::Zero((n_seg + 1) * 4));
        for (int i = 0; i < (n_seg + 1) / 2; i++)
        {
            temp.segment(i * 8, 8) = decvel.segment((i * 2) * 8, 8);
        }
        if (n_seg % 2 != 1)
        {
            temp.tail(4) = decvel.tail(4);
        }

        return temp;
    }

    void minsnapCloseform::calQ()
    {
        /****************************************************************
         *STEP 1.1: Calculate Q
         */
        Q = Eigen::MatrixXd::Zero(n_seg * (n_order + 1), n_seg * (n_order + 1));
    }

    void minsnapCloseform::calM()
    {
        /****************************************************************
         *STEP 1.2: Calculate M
         */
        M = Eigen::MatrixXd::Zero(n_seg * (n_order + 1), n_seg * (n_order + 1));
    }

    void minsnapCloseform::calCt()
    {
        /****************************************************************
         *STEP 1.3: Calculate Ct
         */
        Ct = Eigen::MatrixXd::Zero(n_seg * (n_order + 1), 4 * (n_seg + 1));
    }

    std::pair<Eigen::VectorXd, Eigen::VectorXd> minsnapCloseform::MinSnapCloseFormServer(const Eigen::VectorXd &wp)
    {
        std::pair<Eigen::VectorXd, Eigen::MatrixXd> return_vel;
        // Each matrix is given the final size, for reference only.
        Eigen::VectorXd poly_coef = Eigen::VectorXd::Zero((n_order + 1) * n_seg);
        Eigen::VectorXd dec_vel = Eigen::VectorXd::Zero((n_order + 1) * n_seg);
        Eigen::VectorXd dF(n_seg + 7), dP(3 * (n_seg - 1));
        Eigen::VectorXd start_cond(4), end_cond(4), d(4 * n_seg + 4);
        Eigen::MatrixXd R, R_pp, R_fp;
        start_cond << wp.head(1), 0, 0, 0;
        end_cond << wp.tail(1), 0, 0, 0;

        // Allocate time to each segment of polynomial
        init_ts(1);

        /****************************************************************
         *STEP 1: Calculate three matrices separately in three functions
         */

        calQ();
        calM();
        calCt();

        /****************************************************************
         *STEP 2: Calculate the final matrix R, and separate specific blocks
         */

        // R = ;
        // R_pp = ;
        // R_fp = ;

        /****************************************************************
         *STEP 3: Calculate decision variables and polynomial coefficients
         */

        // dF = ;
        // dP = ;
        // d << dF, dP;
        // dec_vel = ;
        // poly_coef = ;

        return_vel.first = poly_coef;
        return_vel.second = dec_vel;
        return return_vel;
    }

    void minsnapCloseform::calMinsnap_polycoef()
    {
        Eigen::VectorXd wps_x(n_seg + 1), wps_y(n_seg + 1), wps_z(n_seg + 1);
        for (int i = 0; i < n_seg + 1; i++)
        {
            wps_x(i) = wps[i](0);
            wps_y(i) = wps[i](1);
            wps_z(i) = wps[i](2);
        }
        std::pair<Eigen::VectorXd, Eigen::VectorXd> return_vel;
        return_vel = MinSnapCloseFormServer(wps_x);
        poly_coef_x = return_vel.first;
        dec_vel_x = calDecVel(return_vel.second);
        return_vel = MinSnapCloseFormServer(wps_y);
        poly_coef_y = return_vel.first;
        dec_vel_y = calDecVel(return_vel.second);
        return_vel = MinSnapCloseFormServer(wps_z);
        poly_coef_z = return_vel.first;
        dec_vel_z = calDecVel(return_vel.second);
    }

    Eigen::MatrixXd minsnapCloseform::getPolyCoef()
    {
        Eigen::MatrixXd poly_coef(poly_coef_x.size(), 3);
        poly_coef << poly_coef_x, poly_coef_y, poly_coef_z;
        return poly_coef;
    }

    Eigen::MatrixXd minsnapCloseform::getDecVel()
    {
        Eigen::MatrixXd dec_vel(dec_vel_x.size(), 3);
        dec_vel << dec_vel_x, dec_vel_y, dec_vel_z;
        return dec_vel;
    }

    Eigen::VectorXd minsnapCloseform::getTime()
    {
        return ts;
    }

}