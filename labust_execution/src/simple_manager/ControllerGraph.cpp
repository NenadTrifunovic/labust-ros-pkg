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

#include <queue>
#include <list>
#include <algorithm>
#include <sstream>

using namespace labust::control;

ControllerGraph::ControllerGraph():
							pnum(6),
							tnum(0),
							marking(pnum)
{
	//Add the basic vertices.
	std::string dofs[]={"X","Y","Z","K","M","N"};
	//Initial marking
	for (int i=0; i<6;++i)
	{
		placeMap[i]=dofs[i];
		nameMap[dofs[i]].place_num = i;
		marking(i) = 1;
    int k = boost::add_vertex(pngraph);
		pngraph[k].marked = true;
		pngraph[k].type = PNVertexProperty::p;
		pngraph[k].p_num = k;
		pngraph[k].name = dofs[i];
		placeToVertexMap[i] = k;
		resourcePosition.push_back(k);
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

template <typename DistanceMap, typename PredecessorMap>
class bacon_number_recorder : public boost::default_bfs_visitor
{
public:
	bacon_number_recorder(DistanceMap dist, PredecessorMap p) : d(dist), p(p) { }

	template <typename Edge, typename Graph>
	void tree_edge(Edge e, const Graph& g) const
	{
		typename boost::graph_traits<Graph>::vertex_descriptor
		u = source(e, g), v = target(e, g);
		d[v] = d[u] + 1;
		p[v] = u;
	}
private:
	DistanceMap d;
	PredecessorMap p;
};
// Convenience function
template <typename DistanceMap, typename PredecessorMap>
bacon_number_recorder<DistanceMap, PredecessorMap>
record_bacon_number(DistanceMap d, PredecessorMap p)
{
	return bacon_number_recorder<DistanceMap, PredecessorMap>(d,p);
}

void ControllerGraph::get_firing_pn(const std::string& name)
{
	//Desired place to activate
		int des_place = nameMap[name].place_num;

		std::cout<<"Desired place to activate is: "<<placeMap[des_place]<<" == "<<des_place<<std::endl;
		int v = placeToVertexMap[des_place];
		if (pngraph[v].marked)
		{
			std::cout<<"The place is already marked."<<std::endl;
			return;
		}
		else
		{
			std::cout<<"The place is not marked. Searching for firing vector."<<std::endl;
		}

		typedef std::pair<PNGraphType::in_edge_iterator, PNGraphType::in_edge_iterator> RangePair;
		RangePair trans = boost::in_edges(v,pngraph);

		std::cout<<"The place "<<pngraph[v].name<<" has following incoming transitions: ";
		for (PNGraphType::in_edge_iterator it=trans.first; it!=trans.second; ++it)
		{
			int t = boost::source(*it, pngraph);
			std::cout<<pngraph[t].name;
			if (it+1 != trans.second) std::cout<<", ";
		}
		std::cout<<std::endl;

		std::cout<<"Current marking is:";
		print_vec(marking);
		std::cout<<std::endl;

		std::vector<int> markedPlaces;
		for(int i=0; i<marking.size(); ++i)
		{
			if (marking(i))
			{
				markedPlaces.push_back(placeToVertexMap[i]);
			}
		}

		std::map<int, int> pl_dist;
		for(int i=0; i<markedPlaces.size(); ++i)
		{
			//Search the graph
			std::vector <int> bacon_number(boost::num_vertices(pngraph));
			std::vector <int> pred_map(boost::num_vertices(pngraph));
			//Create the distance map
			boost::breadth_first_search(pngraph, markedPlaces[i],
					boost::visitor(record_bacon_number(&bacon_number[0],&pred_map[0])));
			pl_dist[bacon_number[v]] = markedPlaces[i];
		}

		std::vector<int> tokenLocation;
		for(std::set<int>::const_iterator it=pngraph[v].dep_resource.begin(); it!=pngraph[v].dep_resource.end(); ++it)
		{
			int p = resourcePosition[*it];
			tokenLocation.push_back(p);
			std::cout<<"Place "<<pngraph[p].name<<" contains the resource "<<pngraph[*it].name<<std::endl;
		}

	  //Get the transitions from locations
	  std::vector< std::set<int> > firing_seq_vec;
	  tokenLocation = markedPlaces;
	  for(int i=0; i<tokenLocation.size(); ++i)
	  {
	  	std::set<int> firing_seq;
	  	int p = tokenLocation[i];
			//Search the graph
	    std::vector <int> bacon_number(boost::num_vertices(pngraph));
			std::vector <int> pred_map(boost::num_vertices(pngraph));
			//Create the distance map
		  boost::breadth_first_search(pngraph, p,
		                       boost::visitor(record_bacon_number(&bacon_number[0],&pred_map[0])));

	  	std::cout<<"Distance from "<<pngraph[p].name<<" is "<<bacon_number[v]<<std::endl;

	  	if (bacon_number[v] != 0)
	  	{
	  		int pred = v;pred_map[v];
	  		bool validSeq = true;
	  		while (pred != p)
	  		{
	  			if (pngraph[pred].type == PNVertexProperty::t)
	  			{
	  				int tnum = pngraph[pred].t_num, tpair = 0;
	  				if (tnum % 2 == 0) tpair = tnum+1;
	  				else tpair = tnum - 1;
	  				if (firing_seq.find(tpair) != firing_seq.end())
	  				{
	  					std::cout<<"The sequence request adding a T-invariant. The sequence will be ignored."<<std::endl;
	  					validSeq = false;
	  					break;
	  				}
	  				else
	  				{
	  					std::cout<<"Adding transition "<<transitionMap[tnum]<<std::endl;
	  					firing_seq.insert(pngraph[pred].t_num);
	  				}
	  			}
	  			pred = pred_map[pred];
	  		}

	  		if (validSeq) firing_seq_vec.push_back(firing_seq);
	  	}

	  }

	  //Filter firing sequence
	  std::set<int> firing_seq;
	  for (int i=0; i<firing_seq_vec.size(); ++i)
	  {
	  	firing_seq.insert(firing_seq_vec[i].begin(), firing_seq_vec[i].end());
	  }

	  //Process and order the firing sequence.
	  Eigen::VectorXi m = marking;
	  Eigen::VectorXi fire = Eigen::VectorXi::Zero(Dm.cols());
	  bool seq_ok = false;
	  std::cout<<"Triggering the following sequence:";
	  int count = 0; int maxcount = firing_seq.size()+1;
	  bool found_seq = true;
	  while (firing_seq.size())
	  {
	  	std::vector<int> rem;
	  	for(std::set<int>::const_iterator it=firing_seq.begin(); it != firing_seq.end(); ++it)
	  	{
	  		Eigen::VectorXi all_trans = m.transpose()*Dm;
	  		if (all_trans(*it) == Dm.col(*it).sum())
	  		{
	  		  fire(*it) = 1;
	  			m = m + I*fire;
	  			fire(*it) = 0;
	  			rem.push_back(*it);
	  			std::cout<<transitionMap[*it]<<" -> ";
	  		}
	  		else
	  		{
	  			std::cout<<"Transition "<<transitionMap[*it]<<" is not enabled, skipping."<<std::endl;
	  			std::cout<<"Marked places:";
	  			for(int i=0; i<marking.size(); ++i)
	  			{
	  				if (m(i))
	  				{
	  					std::cout<<pngraph[placeToVertexMap[i]].name<<", ";
	  				}
	  			}
	  			std::cout<<std::endl;
	  			++count;
	  		}
	  	}

	  	for(int i=0; i<rem.size();++i)
	  	{
	  		firing_seq.erase(rem[i]);
	  	}

	  	if (count > maxcount)
	  	{
	  		std::cout<<"Unable to find a solution."<<std::endl;
	  		found_seq = false;
	  		break;
	  	}
	  }
	  std::cout<<"end"<<std::endl;

	  marking = m;

	  for(std::set<int>::const_iterator it=pngraph[v].dep_resource.begin(); it!=pngraph[v].dep_resource.end(); ++it)
	  {
	  	resourcePosition[*it] = v;
	  }

	  if (!found_seq) get_firing_pn(name);

/*
	  std::cout<<"Calculating firing seq."<<std::endl;
	  //Firing sequence to closest based on predecessor map.
	  GraphType::vertex_descriptor closest_vx = av_idx[it-av_diff.begin()];
	  GraphType::vertex_descriptor curr_pred = closest_vx;

	  boost::property_map<GraphType, boost::edge_name_t>::type edgeMap =
	  		boost::get(boost::edge_name_t(), rgraph);
	  std::cout<<"Get the edge map."<<std::endl;
	//  if (boost::edge(curr_pred, closest_vx, rgraph).second)
	//  {
	//  	std::cout<<"Get edge connection between "<<curr_pred<<" and "<<closest_vx<<std::endl;
	//    firing_seq.push_back(edgeMap[boost::edge(curr_pred, closest_vx, rgraph).first]);
	//  }
	//  else
	//  {
	//  	std::cout<<"No edge connection between "<<curr_pred<<" and "<<closest_vx<<std::endl;
	//  }

	  std::cout<<"Loop across edges. The curr_pred = closest_vx = "<<closest_vx<<std::endl;
	  //Go back...
	  std::vector<int> firing_seq;
	  while (curr_pred != curr_vx)
	  {
	    if (boost::edge(pred_map[curr_pred], curr_pred, rgraph).second)
	    {
	    	std::cout<<"Get edge connection between "<<pred_map[curr_pred]<<" and "<<curr_pred<<std::endl;
	    	firing_seq.push_back(edgeMap[boost::edge(pred_map[curr_pred], curr_pred, rgraph).first]);
	    }
	    else
	    {
	    	std::cout<<"No edge connection between "<<pred_map[curr_pred]<<" and "<<curr_pred<<std::endl;
	    }
	    curr_pred = pred_map[curr_pred];
	  }

	  std::cout<<"Triggering the following sequence:";
	  for (int i=firing_seq.size()-1; i>=0; --i)
	  {
	  	std::cout<<transitionMap[firing_seq[i]]<<" -> ";
	  }
	  std::cout<<"end"<<std::endl;

	  marking = m_new;
	  */
}

void ControllerGraph::addToPNGraph(const navcon_msgs::RegisterController_v3Request& info)
{
	typedef PNGraphType::vertex_descriptor IntType;
	PlaceInfo& newcon=nameMap[info.name];

	IntType place = boost::add_vertex(pngraph),
			en_t = boost::add_vertex(pngraph),
			dis_t = boost::add_vertex(pngraph);

	pngraph[place].type = PNVertexProperty::p;
	pngraph[place].p_num = newcon.place_num;
	pngraph[place].marked = false;
	pngraph[place].name = placeMap[newcon.place_num];
	placeToVertexMap[newcon.place_num] = place;

	pngraph[en_t].type = PNVertexProperty::t;
	pngraph[en_t].t_num = newcon.enable_t;
	pngraph[en_t].name = transitionMap[newcon.enable_t];

	pngraph[dis_t].type = PNVertexProperty::t;
	pngraph[dis_t].t_num = newcon.disable_t;
	pngraph[dis_t].name = transitionMap[newcon.disable_t];

	//Add local connections
	boost::add_edge(en_t, place, 1, pngraph);
	boost::add_edge(place, dis_t, 1, pngraph);

	//Add basic dependencies.
	for (int i=0; i<info.used_tau.size(); ++i)
	{
		if (info.used_tau[i])
		{
			boost::add_edge(i,en_t,1,pngraph);
			boost::add_edge(dis_t,i,1,pngraph);
			pngraph[place].dep_resource.insert(i);
		}
	}

	//Add advanced dependencies
	for (int i=0; i<info.used_other.size(); ++i)
	{
		PNGraphType::vertex_descriptor place_num =
				placeToVertexMap[nameMap[info.used_other[i]].place_num];
		pngraph[place].dep_resource.insert(pngraph[place_num].dep_resource.begin(), pngraph[place_num].dep_resource.end());
		boost::add_edge(place_num, en_t,1,pngraph);
		boost::add_edge(dis_t, place_num,1,pngraph);
	}
}

void ControllerGraph::getDotDesc2(std::string& desc)
{
	using namespace boost;
	//Construct a label writer.
	std::ostringstream out;
	write_graphviz(out, pngraph,
			pn_writer2(pngraph));
	desc = out.str();
}

