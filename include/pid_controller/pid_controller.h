/**
 * MIT License
 *
 * Copyright (c) 2020 Ryan Shedlock
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file PID_Controller.h
 * @brief Base class for Controls library.
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <stddef.h>
#include <string.h>

namespace controls
{
    class PID_Controller
    {
        public:

        /**
         * @brief      Constructs a new instance.
         *
         * @param[in]  _kp        The Porportional Gain Term
         * @param[in]  _ki        The Integral Gain Term
         * @param[in]  _kd        The Differential Gain Term
         * @param[in]  _cmdmax    The Positive Saturation Limit for the PID
         * @param[in]  _cmdmin    The Negative Saturation Limit for the PID
         * @param[in]  _imax      The Positive Saturation Limit for the I Term
         * @param[in]  _imin      The Negative Saturation Limit for the I Term
         * @param[in]  _init_val  The Initial Value for the Commanded PID Output
         * @param[in]  _fltr_coef The Time Constant for D Term 1st Order LPF
         */
        PID_Controller(double &_kp, double &_ki, double &_kd, double &_cmdmax, double &_cmdmin, double &_imax, double &_imin, double &_init_val, double &_fltr_coef);

        /**
         * @brief      Destroys the object.
         */
        ~PID_Controller();

        /**
         * @brief      Initializes the PID Controller
         */
        void Init();

        /**
         * @brief      Cyclic Execution Step of the PID (aka RUN PID)
         * 
         * @return     Controller Output Command
         */
        double Step(double &_error);

        /**
         * @brief      Dynamically Update Controller Gains
         */
        void Update_Gains(double &_kp, double &_ki, double &_kd);

        /**
         * @brief      Dynamically Update Time Step
         */
        void Update_dT(double &_dt);

        /**
         * @brief      Dynamically Controller Initial Value
         */
        void Update_InitVal(double &_init_val);

        /**
         * @brief      Dynamically Update D-Term LPF Time Constant
         */
        void Update_D_Filter(double &_fltr_coef);

        /**
         * @brief      Dynamically Update PID Controller Saturation Limits
         */
        void Update_Sat_Limit(double &_cmdmax, double &_cmdmin);

        /**
         * @brief      Dynamically Update I Term Saturation Limits
         */
        void Update_I_Sat_Limit(double &_imax, double &_imin);

        /**
         * @brief      Dynamically Reset the Integrator
         */
        void Reset_I();

        /**
         * @brief      Dynamically Reset the PID Commanded Output
         */
        void Reset_PID();

        /**
         * @brief      Trun-On the PID Controller
         */
        void Enable();

        /**
         * @brief      Turn-Off the PID Controller
         */
        void Disable();

        /**
         * @brief      Get PID Saturation Status (TRUE == Saturated)
         */
        bool Saturated();

        /**
         * @brief      Get I Term Saturation Status (TRUE == Saturated)
         */
        bool I_Saturated();

        /* PID Terms/Params */
        double error_;
        double cmd_;
        double pout_;
        double iout_;
        double dout_;
        double kp_;
        double ki_;
        double kd_;
        double imax_;
        double imin_;
        double cmdmax_;
        double cmdmin_;
        double fltr_coef_;
        double init_val_;
        double dt_;
        bool reset_;
        bool enabled_;
        bool isat_;
        bool pidsat_;

        private:
        double err_unitdelay_;
        double i_unitdelay_;
        double dfltr_unitdelay_;
        double t_minus_;
        /* Discrete Time Unit Delay (1/z) Data Stores */
        
    };
} // namespace controls

#endif /* PID_CONTROLLER_H_ */