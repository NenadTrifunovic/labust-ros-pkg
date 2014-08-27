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
#ifndef CONTROLLERGRAPH_HPP_
#define CONTROLLERGRAPH_HPP_
#include <labust/graph/BFSTools.hpp>
#include <labust/graph/PNGraph.hpp>

#include <navcon_msgs/RegisterController_v3.h>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <Eigen/Dense>

#include <string>
#include <map>
#include <set>

namespace labust
{
	namespace control
	{

		/**
		 * The controller graph interface. Different graphs and graph search solutions should implement this interface
		 * in order to be easily used by the ROS node.
		 */
		class ControllerGraphInterface
		{
		public:
			virtual ~ControllerGraphInterface(){};

			/**
			 *
			 * @param info
			 /*
			  * Adds the new controller to the controller list and creates a vertex in the graph.
			  * @param info Controller dependency and name information
			  * @return The service reply value depending on success or reason of failure.
			  */
			virtual int addToGraph(const navcon_msgs::RegisterController_v3Request& info) = 0;

			virtual void addResource(const std::string& name) = 0;
		};

		/**
		 * The class contains implementation of Petri-Net builder and controller.
		 * \todo Add multiple desired places setup
		 * \todo Add place turn-off option
		 * \todo Add additional state enabled place in parallel with the control place.
		 * \todo Add weighted transitions for multi DOF controllers ?
		 * \todo Add simulation step for activation to detect wrong firing sequences.
		 * \todo Extract reachability graph building and ploting
		 * \todo Extract debugging information and structure it in a debug class
		 * \todo Possible optimizations: Incremental graph building
		 * \todo Possible optimizations: BFS only on a subset of the graph ?
		 * \todo Possible optimizations: Spin-off BFS for the new state in a thread to have it ready in advance
		 * \todo Recommended optimizations: Reachability calculation memory/time/copying/std::find usage
		 * \todo Add detection of faulty controller registrations or setups.
		 *       -analyze if they have indirect dependencies to DOFs
		 * \todo Possible optimizations: Find sequences that can fire simultaneously
		 * \todo Revisit includes
		 */
		class ControllerGraph : public virtual ControllerGraphInterface
		{
			typedef labust::graph::PNGraph::VertexProperty PNIdx;
			/**
			 * Global information holder for controller.
			 */
			struct ControllerInfo
			{
				ControllerInfo(){};

				///Controller place index
				PNIdx	place;
				///Enabling transition index
				PNIdx enable_t;
				///Disabling transition index
				PNIdx disable_t;
				///Activation indicator index
				PNIdx indicator;

				///Base resource dependency tracking
				std::set<std::string> dep_resources;
			};

		public:
			/**
			 * Main constructor
			 */
			ControllerGraph();

			/**
			 * Get the firing sequence for the named controller by reverse graph search.
			 */
			void get_firing_pn(const std::string& name);
			/**
			 * Adds the new controller to the controller list and creates a vertex in the graph.
			 * @param info
			 */
			int addToGraph(const navcon_msgs::RegisterController_v3::Request& info);

			void addResource(const std::string& name);
			/**
			 * Get the reachability graph DOT description.
			 */
			void getDotDesc2(std::string& desc);


			typedef std::map<std::string, navcon_msgs::RegisterController_v3::Request> ControllerMap;
			/**
			 * The registered controllers.
			 */
			ControllerMap controllers;

		private:

			/**
			 * The last firing sequence.
			 */
			std::vector<int> firing_seq;
			/**
			 * The current resource position.
			 */
			std::map<std::string, std::string> resourcePosition;


			///The Petri-Net control graph
			labust::graph::PNGraph pngraph;
			///The Petri-Net general graph
			labust::graph::PNGraph fullgraph;
			///Basic resources list
			std::map<std::string, ControllerInfo> baseResources;
			///Helper name to p/t mapping.
			std::map<std::string, ControllerInfo> nameMap;
		};
	}
}

/* CONTROLLERGRAPH_HPP_ */
#endif
