/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, LABUST, UNIZG-FER
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LABUST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <math.h>
#include <ros/ros.h>

void PIFF_modelTune(PIDBase* self,
		const PT1Model* const model,
		float w)
{
	self->Kp = 2*w*model->alpha-model->beta;
	self->Ki = model->alpha*w*w;
	self->Kd = self->Kt = self->Tf = 0;

	//The empirical parameter for overshoot ~5%
	float a=1.5;
	self->b = a*self->Ki/(self->Kp*w);
	ROS_ERROR("b-value dyn: %f %f",self->b,w);
	//self->b = 1;

	self->model.alpha = model->alpha;
	self->model.beta = model->beta;
	self->model.betaa = model->betaa;

	self->w = w;
}

void PIFF_tune(PIDBase* self, float w)
{
	self->Kp = 2*w;
	self->Ki = w*w;
	self->Kd = self->Kt = self->Tf = 0;
	//The empirical parameter for overshoot ~5%
	float a=1.5;
	self->b = a*self->Ki/(self->Kp*w);
	//self->b = 1;
	ROS_ERROR("b-value kin: %f %f",self->b,w);

	self->w = w;
}

void PIFF_wffIdle(PIDBase* self, float Ts, float error, float perror, float ff)
{
	self->lastError = error;
	self->lastPError = perror;
	self->lastRef = self->desired;
	self->lastFF = ff;
	self->lastState = self->state;
	self->internalState = self->Kp*perror;
	self->internalState += ff;
	self->I = self->track - self->internalState;
	self->output = self->internalState = self->track;
}

void PIFF_wffStep(PIDBase* self, float Ts, float error, float perror, float ff)
{
	//Perform windup test if automatic mode is enabled.
	if (self->autoWindup == 1)
	{
		//self->windup = (self->internalState > self->output) && (error>0);
		//self->windup = (self->windup) ||
		//		((self->internalState < self->output) && (error<0));
		//Set the wind-up sign for cascade controllers.
		///\todo Deprecate windup sign ?
		self->track = self->output;
		self->windup = (self->internalState != self->track) && (error*self->track > 0);
	}
	else
	{
		//Experimental
		/*self->windup = ((self->extWindup > 0) && (error > 0)) ||
				((self->extWindup < 0) && (error < 0));*/
		//Experimental 2
		self->windup = (self->extWindup && (error*self->output > 0));
	}

	//Backward recalculation
	if ((self->lastI != 0) && self->windup && self->useBackward)
	{
		//Calculate the proportional influence
		//float diff = self->track - self->internalState + self->lastI;
		//If the proportional part is already in windup remove the whole last integral
		//Otherwise recalculate the integral to be on the edge of windup
		//self->internalState -= ((diff*self->track <= 0)?self->lastI:(self->lastI - diff));
	}

	//Proportional term
	//self->internalState += self->Kp*(error-self->lastError);
	//self->internalState += self->Kp*(perror - self->lastPError);
	self->internalState = self->Kp*perror;
	//Integral term
	//Disabled if windup is in progress.
	//if (!self->windup) self->internalState += (self->lastI = self->Ki*Ts*error);
	//else self->lastI = 0;
	//Feed forward term
	//self->internalState += ff - self->lastFF;
	self->internalState += ff;
	if (!self->windup) self->I += (self->lastI = self->Ki*Ts*error);
	else
	{
		//self->I = self->track - self->internalState;
		float diff = self->track - self->internalState;
		if (diff*self->track <= 0)
		{
			self->I = 0;
		}
		else
		{
			self->I = diff;
		}
	}
	self->internalState += self->I;

	//Set final output
	self->output = self->internalState;

	if (self->autoWindup == 1)
	{
		self->output = sat(self->output,-self->outputLimit, self->outputLimit);
	}

	self->lastError = error;
	self->lastPError = perror;
	self->lastRef = self->desired;
	self->lastFF = ff;
	self->lastState = self->state;
}

