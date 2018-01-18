/*********************************************************************
 * xmlPrinter.hpp
 *
 *  Created on: Apr 3, 2014
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2016, LABUST, UNIZG-FER
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

#ifndef XMLPRINTER_HPP_
#define XMLPRINTER_HPP_

#include <labust/mission/labustMission.hpp>
#include <labust/primitive/PrimitiveMapGenerator.h>
#include <boost/lexical_cast.hpp>
#include <tinyxml2.h>
#include <vector>

using namespace tinyxml2;
using namespace std;

namespace labust
{
	namespace utils
	{
		/*****************************************************************
		 *** WriteXML Class definition
		 ****************************************************************/

		class WriteXML
		{

		public:

			/************************************************************
			 *** Class functions
			 ************************************************************/

			WriteXML(const std::string& xml);

			void addMission();

			void addEvent();

			void saveXML(string fileName);

			void addXMLNode(XMLNode* parentNode, string nodeName, string attrName, string attrValue, string value);

			void addPrimitive(int primitive_id, vector<string> primitive_data);

			/************************************************************
			 *** Class variables
			 ************************************************************/

		private:

			labust::primitive::PrimitiveMapGenerator PP;

			XMLDocument doc;

			XMLNode *mission;
			XMLNode *main;
			XMLNode *primitive;
			XMLNode *param;
			XMLNode *idNode;
			XMLNode *events;

			int id;
		};


		WriteXML::WriteXML(const std::string& xml):id(0),PP(xml)
		{
			primitive = doc.NewElement("");
			events = doc.NewElement("");
			idNode = doc.NewElement("");
			mission = doc.NewElement("");
			param = doc.NewElement("");
			main = doc.NewElement("");
		}

		void WriteXML::addMission(){
			main = doc.NewElement("main");
			mission = doc.NewElement("mission");
			doc.InsertEndChild(main);
			main->InsertEndChild(mission);
			id = 0;
		}

		void WriteXML::addEvent()
		{
			events = doc.NewElement("events");
			doc.InsertEndChild(events);
		}

		void WriteXML::saveXML(string fileName)
		{
			doc.SaveFile(fileName.c_str());
			doc.Clear();
		}

		void WriteXML::addXMLNode(XMLNode* parentNode, string nodeName, string attrName, string attrValue, string value)
		{
			XMLNode *node;
			node = doc.NewElement(nodeName.c_str());
			if(attrName.empty() == 0)
				node->ToElement()->SetAttribute(attrName.c_str(),attrValue.c_str());
			node->InsertEndChild(doc.NewText(value.c_str()));
			parentNode->InsertEndChild(node);
		}

		void WriteXML::addPrimitive(int primitive_id, std::vector<std::string> primitive_data)
		{
			id++;
			primitive = doc.NewElement("primitive");
			primitive->ToElement()->SetAttribute("name",PRIMITIVES[primitive_id]);

			addXMLNode(primitive,"id","","",boost::lexical_cast<std::string>(id));

			std::vector<std::string>::size_type k = 0;
			for(std::vector<std::string>::iterator it = PP.primitive_params[primitive_id].begin(); it != PP.primitive_params[primitive_id].end(); ++it)
			{
				addXMLNode(primitive,"param","name",(*it).c_str(),primitive_data.at(k).c_str());
				k++;
			}
			mission->InsertEndChild(primitive);
		}
	}
}

#endif /* XMLPRINTER_HPP_ */
