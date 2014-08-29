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
#ifndef LEGACYCOMPAT_HPP_
#define LEGACYCOMPAT_HPP_
#include <map>
#include <string>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the configuration for legacy controller infrastructure
		 * compatibility with Petri-Net execution framework.
		 */
		class LegacyCompat
		{
			typedef std::map<std::string, int> LLMap;
			typedef std::map<std::string, std::string>	HLMap;
		public:
			/**
			 * Main constructor
			 */
			LegacyCompat();

			/**
			 * Configure the Legacy compat service clients.
			 */
			void onInit(){};

			/**
			 * Call a single legacy service based on name and state.
			 */
			bool callService(const std::string& name, int state);

		private:
			///Low-level to index mapping
			LLMap lowLevel;
			///Identification to index mapping
			LLMap ident;
			///High-level to service name mapping
			HLMap hlLevel;
		};
	}
}

/* LEGACYCOMPAT_HPP_ */
#endif
