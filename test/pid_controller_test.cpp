#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "pid_controller/pid_controller.h"


namespace controls 
{

    class PID_Test : public ::testing::Test 
    {
        protected:
        /// Pointer for Kalman Filter object
        std::shared_ptr<PID_Controller> pid_ptr;
        double sim_error;
        double kp, ki, kd;
        double max, min;
        double imax, imin;
        double init_val;
        double fltr_coef;

        virtual void SetUp()
        {
            kp = 0.0F;
            ki = 0.0F;
            kd = 0.0F;
            max = 10.0F;
            min = -max;
            imax = 3.0F;
            imin = -imax;
            init_val = 0.0F;
            fltr_coef = 1.0F;
            pid_ptr = std::make_shared<PID_Controller>(kp, ki, kd, max, min, imax, imin, init_val, fltr_coef);
        }

        virtual void TearDown()
        {
        }
    };

    TEST_F(PID_Test, PTEST)
    {
        double kp = 3.0F;
        double ki = 0.0F;
        double kd = 0.0F;
        double dT = 0.1F;
        bool result = false;

        Eigen::VectorXd error(63);          /// Control Error
        Eigen::VectorXd pid_expected(63);   /// Expected 
        Eigen::VectorXd pid_out(63);        /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;                  /// Mean Abs Error
        
        pid_ptr->Init();
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 5.17 deg/100ms error signal
        for(int i=0; i<63; i++)
        {      
            error(i) = sin(i*dT);      
            pid_expected(i) = kp*error(i);
            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
        }
        
        /// Mean Absolute Error
        mae /= 63;
        if(fabs(mae) < 1e-6)
        {
            result = true;
        }
        else 
        {
            result = false;
            std::cerr << "PTest Mean Absolute Arror: " << mae << std::endl;
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, DTEST)
    {
        double kp = 0.0F;
        double ki = 0.0F;
        double kd = 1.0F;
        double dT = 0.01F;
        bool result = false;

        Eigen::VectorXd error(630);          /// Control Error
        Eigen::VectorXd pid_expected(630);   /// Expected 
        Eigen::VectorXd pid_out(630);        /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;
        double rmse = 0.0F;                  /// Mean Abs Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<630; i++)
        {
            error(i) = sin(i*dT);

            pid_expected(i) = kd*cos(i*dT);
            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
            cmp_error = cmp_error * cmp_error;
            rmse += cmp_error;
        }

        /// Mean Absolute Error
        mae /= 630;
        rmse /= 630;
        rmse = sqrt(rmse);
        //std::cerr << "DTest RMSE " << rmse << std::endl;
        //std::cerr << "DTest MAE " << mae << std::endl;
        if(fabs(mae) < 0.002)
        {
            result = true;
        }
        else 
        {
            result = false;
            std::cerr << "DTest Mean Absolute Error: " << mae << std::endl;
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, ITEST)
    {
        double kp = 0.0F;
        double ki = 1.0F;
        double kd = 0.0F;
        double dT = 0.01F;
        double imax = 5.0F;
        double imin = -5.0F;
        double initial_val = 0.0F;
        bool result = false;

        Eigen::VectorXd error(630);          /// Control Error
        Eigen::VectorXd pid_expected(630);   /// Expected 
        Eigen::VectorXd pid_out(630);        /// Controller Output
        double cmp_error = 0.0F;            /// Compare Error
        double mae = 0.0F;                  /// Mean Abs Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_I_Sat_Limit(imax, imin);
        pid_ptr->Update_InitVal(initial_val);
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<630; i++)
        {
            error(i) = sin(i*dT);

            pid_expected(i) = ki*1.0F-cos(i*dT);
            pid_out(i) = pid_ptr->Step(error(i));
            cmp_error = pid_expected(i) - pid_out(i);
            mae += cmp_error;
        }

        /// Mean Absolute Error
        mae /= 630;
        if(fabs(mae) < 0.002)
        {
            result = true;
        }
        else 
        {
            result = false;
            std::cerr << "ITest Mean Absolute Error: " << mae << std::endl;
        }
        EXPECT_TRUE(result);
    }

    TEST_F(PID_Test, ISAT_TEST)
    {
        double kp = 0.0F;
        double ki = 1.0F;
        double kd = 0.0F;
        double dT = 0.01F;
        double imax = 0.5F;
        double imin = 0.5F;
        double initial_val = 0.0F;
        bool result = false;

        double pid_out;        /// Controller Output
        double error;          /// Controller Error

        /// Initialize Controller
        pid_ptr->Init();
        pid_ptr->Update_I_Sat_Limit(imax, imin);
        pid_ptr->Update_InitVal(initial_val);
        pid_ptr->Update_Gains(kp, ki, kd);
        pid_ptr->Update_dT(dT);

        /// Run Controler with 0.57 deg/10ms error signal
        for(int i=0; i<1260; i++)
        {
            error = sin(i*dT);
            pid_out = pid_ptr->Step(error);
            if(pid_out > 0.5F || pid_out < -0.5F)
            {
                result = false;
                std::cerr << "PID_OUT: " << pid_out << "   I LIM +/-: " 
                          << imax << std::endl;
            }
        }

        EXPECT_TRUE(result);
    }
};

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}