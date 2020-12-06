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
 * @file PID_Controller.cpp
 * @brief Base class for Controls library.
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */

#include "pid_controller/pid_controller.h"

namespace controls
{
    void PID_Controller::Init()
    {
        reset_ = false;
        isat_ = false;
        pidsat_ = false;
        pout_ = 0.0F;
        iout_ = 0.0F;
        dout_ = 0.0F;
        error_ = 0.0F;
        fltr_coef_ = 1.0F;
        i_unitdelay_ = 0.0F;
        err_unitdelay_ = 0.0F;
        dfltr_unitdelay_ = 0.0F;
        err_unitdelay_ = 0.0F;
        cmd_= init_val_;
    }

    void PID_Controller::Update_Gains(double &_kp, double &_ki, double &_kd)
    {
        kp_ = _kp;
        ki_ = _ki;
        kd_ = _kd;
    }

    void PID_Controller::Update_dT(double &_dt)
    {
        dt_ = _dt;
    }

    void PID_Controller::Update_Sat_Limit(double &_cmdmax, double &_cmdmin)
    {
        cmdmax_ = _cmdmax;
        cmdmin_ = _cmdmin;
    }

    void PID_Controller::Update_I_Sat_Limit(double &_imax, double &_imin)
    {
        imax_ = _imax;
        imin_ = _imin;
    }

    void PID_Controller::Update_D_Filter(double &_fltr_coef)
    {
        fltr_coef_ = _fltr_coef;
    }
    
    void PID_Controller::Update_InitVal(double &_init_val)
    {
        init_val_ = _init_val;
    }


    PID_Controller::PID_Controller(double &_kp, double &_ki, double &_kd, double &_cmdmax, double &_cmdmin, double &_imax, double &_imin, double &_init_val, double &_fltr_coef)
    {
        enabled_ = true;
        init_val_ = _init_val;
        this->Init();
        this->Update_Gains(_kp, _ki, _kd);
        this->Update_Sat_Limit(_cmdmax, _cmdmin);
        this->Update_I_Sat_Limit(_imax, _imin);
        this->Update_D_Filter(_fltr_coef);
    }

    PID_Controller::~PID_Controller()
    {
    }

    double PID_Controller::Step(double &_error)
    {
        error_ = _error;
        double tmp = 0;
        double tmp_d = 0;

        if(enabled_)
        {
            if(~reset_)
            {
                /* Calculate Integral Gain, Apply Limits */
                iout_ = error_ * ki_ * dt_ + i_unitdelay_;
                i_unitdelay_ = iout_;
                if(iout_ > imax_)
                {
                    iout_ = imax_;
                    isat_ = true;
                }
                else if(iout_ < imin_)
                {
                    iout_ = imin_;
                    isat_ = true;
                }
                else
                {
                    isat_ = false;
                }

                /* Calculate Differential Gain, Apply 1st Order Digital LPF */
                tmp_d = ((error_  - err_unitdelay_) * kd_) / dt_;
                dout_ = dfltr_unitdelay_ + ((tmp_d - dfltr_unitdelay_) * fltr_coef_);
                dfltr_unitdelay_ = tmp_d;

                /* Calculate Porportional Gain */
                pout_ = error_ * kp_;

                /* Calculate PID Controller Command, Apply Limits */
                cmd_= pout_ + iout_ + dout_; 
                if(cmd_> cmdmax_)
                {
                    cmd_= cmdmax_;
                    pidsat_ = true;
                }
                else if(cmd_< cmdmin_)
                {
                    cmd_= cmdmin_;
                    pidsat_ = true;
                }
                else
                {
                    pidsat_ = false;
                }

            }
            else 
            {
                /* If Reset, Re-initialize Controller, Set cmd_to Init Value */
                reset_ = false;
                i_unitdelay_ = 0.0F;
                err_unitdelay_ = 0.0F;
                dfltr_unitdelay_ = init_val_;
                error_ = 0.0F;
                err_unitdelay_ = 0.0F;
                cmd_= init_val_;
            }
        }
        else 
        {
            this->Init();
            cmd_= 0.0F;
        }

        err_unitdelay_ = error_;
        return cmd_;
    }

    void PID_Controller::Reset_I()
    {
        iout_ = 0.0F;
        i_unitdelay_ = 0.0F;
    }
    void PID_Controller::Reset_PID()
    {
        reset_ = true;
    }
    void PID_Controller::Enable()
    {
        enabled_ = true;
    }
    void PID_Controller::Disable()
    {
        enabled_ = false;
    }
    bool PID_Controller::Saturated()
    {
        return pidsat_;
    }
    bool PID_Controller::I_Saturated()
    {
        return isat_;
    }
}