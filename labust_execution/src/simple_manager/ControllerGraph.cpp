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
 *
 *  Author: Dula Nad
 *  Created: 30.10.2013.
 *********************************************************************/
#include <labust/control/ControllerGraph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <ros/ros.h>

#include <queue>
#include <list>
#include <algorithm>
#include <sstream>

using namespace labust::control;

ControllerGraph::ControllerGraph()
{
	//Add the basic vertices.
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Initial marking
	for (int i=0; i<6;++i)
	{
	/*	placeMap[i]=dofs[i];
		nameMap[dofs[i]].place_num = i;
		marking(i) = 1;
    int k = boost::add_vertex(pngraph);
		pngraph[k].marked = true;
		pngraph[k].type = PNVertexProperty::p;
		pngraph[k].p_num = k;
		pngraph[k].name = dofs[i];
		placeToVertexMap[i] = k;
		resourcePosition.push_back(k);*/
	}
}

template <class Vector>
void print_vec(const Vector& vec)
{
	std::cout<<"(";
	int i;
	for(i=0; i<vec.size()-1;++i)
	{
		std::cout<<vec(i)<<",";
	}
	std::cout<<vec(i)<<")";
}


void ControllerGraph::get_firing_pn(const std::string& name)
{
	using namespace labust::graph;
	//Desired place to activate
	PNIdx des_place = nameMap[name].place;
	int v = des_place.vertexIdx;

	ROS_INFO("Desired place to activate is: %s == %s", des_place.name.c_str(), name.c_str());

	if (pngraph.isMarked(des_place))
	{
		ROS_INFO("The place is already marked.");
		return;
	}
	else
	{
		ROS_INFO("The place is not marked. Searching for firing vector.");
	}

	typedef std::pair<PNGraph::GraphType::in_edge_iterator,
			PNGraph::GraphType::in_edge_iterator> RangePair;
	RangePair trans = boost::in_edges(des_place.vertexIdx,pngraph.pngraph);

	ROS_INFO("The place %s has following incoming transitions: ",des_place.name.c_str());
	for (PNGraph::GraphType::in_edge_iterator it=trans.first;
			it!=trans.second; ++it)
	{
		int t = boost::source(*it, pngraph.pngraph);
		ROS_INFO("  %s",pngraph.pngraph[t].name.c_str());
	}

	std::cout<<"Current marking is:"<<pngraph.marking.transpose().eval()<<std::endl;

	std::vector<PNIdx> markedPlaces;
	for(int i=0; i<pngraph.marking.size(); ++i)
	{
		if (pngraph.marking(i))
		{
			int vertex = pngraph.placeToVertex[i];
			markedPlaces.push_back(pngraph.pngraph[vertex]);
		}
	}

	///\todo Add only a single calculation of the distance graph
	///\todo Pass the whole vertex description if needed
	//Get the transitions from locations
	std::vector< PNGraph::TSequencePtr > firing_seq_vec;
	//tokenLocation = markedPlaces;
	int maxlen(0), maxidx(0);
	for(int i=0; i<markedPlaces.size(); ++i)
	{
		//std::set<int> firing_seq;
	  //int p = tokenLocation[i];
		PNGraph::TSequencePtr seq = pngraph.findTransitions(markedPlaces[i], des_place, false);

		if (seq->size() > 0) firing_seq_vec.push_back(seq);

	  if (seq->size() > maxlen)
	  {
	  	maxidx = i;
	  	maxlen = seq->size();
	  }
	}

	std::cout<<"Transitions OK. Size:"<<maxlen<<std::endl;

	//Pad firing sequences to same size
	for(int i=0; i<firing_seq_vec.size(); ++i)
	{
		PNGraph::TSequencePtr seq = firing_seq_vec[i];
		int len = seq->size();
		PNIdx dummy;
		dummy.idx = -1;
		if ((maxlen - len) > 0)
			seq->insert(seq->end(), maxlen-len,dummy);
	}

	std::cout<<"Transitions paded."<<std::endl;

	//Extract unique sequence
	PNGraph::TSequence unique;
	for (int i=0; i<maxlen; ++i)
	{
		std::set<int> firingSet;
		for(int j=0; j<firing_seq_vec.size(); ++j)
		{
			if (firing_seq_vec[j]->at(i).idx != -1)
				firingSet.insert(firing_seq_vec[j]->at(i).vertexIdx);
		}

		for (std::set<int>::iterator it = firingSet.begin();
					it != firingSet.end();
					++it)
		{
			unique.push_back(pngraph.pngraph[*it]);
		}
	}

	std::cout<<"Unique sequence: ";
	for (PNGraph::TSequence::reverse_iterator it = unique.rbegin(); it != unique.rend(); ++it)
	{
		std::cout<<it->name<<" -> ";
	}
	std::cout<<"end"<<std::endl;

	///\todo Resolve the iterative approach:
	///When labeling is available each place that contains more than one required label
	///should be disabled first before searching for a solution
	///Alternatively the graph search could go from the source place and take into account
	///the transitions that have to be avoided
	///Third option, without labeling, identify the places that contain tokens of interest
	///and if they have more tokens first disable them.
	std::reverse(unique.begin(), unique.end());
	if (!pngraph.fire(unique)) get_firing_pn(name);

	std::cout<<"Current marking is:"<<pngraph.marking.transpose().eval()<<std::endl;
}


void ControllerGraph::addResource(const std::string& name)
{
	//Create base resource
	ControllerInfo& resource = baseResources[name];
	resource.place = pngraph.addPlace(name);
	pngraph.addToken(resource.place, name);

	//\todo Is this required ?
	nameMap[name] = resource;
}

int ControllerGraph::addToGraph(const navcon_msgs::RegisterController_v3::Request& info)
{
	///\todo Check for un-met dependencies
	///\todo Dismiss controller registration if double or unmet dependencies
	///\todo Add the accepted controller to the Petri-Net and dependency graphs

	//Check if the controller with same name is already registered.
	if (nameMap.find(info.name) != nameMap.end())
	{
		ROS_WARN("Controller with name %s already exists.", info.name.c_str());
		return navcon_msgs::RegisterController_v3::Response::ALREADY_REGISTERED;
	}

	//Check if resources are available
	for (int i=0; i<info.used_resources.size(); ++i)
	{
		if (nameMap.find(info.used_resources[i]) == nameMap.end())
		{
			ROS_ERROR("The controller %s is missing dependency %s.",
					info.name.c_str(),
					info.used_resources[i].c_str());
			return navcon_msgs::RegisterController_v3::Response::MISSING_DEPENDENCY;
		}
	}

	//Create new controller info
	ControllerInfo& newcon=nameMap[info.name];
	//Add places and transitions to the controller net
  newcon.place = pngraph.addPlace(info.name);
  newcon.enable_t = pngraph.addTransition(info.name+"_enable");
  newcon.disable_t = pngraph.addTransition(info.name+"_disable");
	//Connect the new additions
  pngraph.connect(newcon.place, newcon.disable_t);
  pngraph.connect(newcon.enable_t, newcon.place);

	//Add connections to required resources
	for (int i=0; i<info.used_resources.size(); ++i)
	{
		ControllerInfo& dep = nameMap[info.used_resources[i]];
		pngraph.connect(dep.place, newcon.enable_t);
		pngraph.connect(newcon.disable_t, dep.place);

		if (baseResources.find(info.used_resources[i]) == baseResources.end())
		{
			newcon.dep_resources.insert(dep.dep_resources.begin(),
					dep.dep_resources.end());
		}
		else
		{
			newcon.dep_resources.insert(info.used_resources[i]);
		}
	}

	//Save temporary representation of the graph
	std::fstream pn_file("pn_graph.dot",std::ios::out);
	pn_file<<pngraph;

	return navcon_msgs::RegisterController_v3::Response::SUCCESS;
}

