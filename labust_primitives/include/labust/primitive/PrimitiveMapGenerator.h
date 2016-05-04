//TODO Check if parameters are loaded properly.

/*********************************************************************
 * PrimitiveMapGenerator.h
 *
 *  Created on: Apr 29, 2016
 *      Author: Filip Mandic
 *
 ********************************************************************/

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, LABUST, UNIZG-FER
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

#ifndef LABUST_ROS_PKG_LABUST_PRIMITIVES_INCLUDE_LABUST_PRIMITIVE_PRIMITIVEMAPGENERATOR_H_
#define LABUST_ROS_PKG_LABUST_PRIMITIVES_INCLUDE_LABUST_PRIMITIVE_PRIMITIVEMAPGENERATOR_H_

#include <labust/mission/labustMission.hpp>
#include <tinyxml2.h>

using namespace tinyxml2;

namespace labust
{
	namespace primitive
	{
		class PrimitiveMapGenerator
		{
		public:

			PrimitiveMapGenerator(string path):xml_path(path)
			{
				generatePrimitiveData(xml_path);

			}

			~PrimitiveMapGenerator()
			{

			}

			map<string, bool> getPrimitiveBoolMap()
			{
				return bool_map;
			}

			map<string, double> getPrimitiveDoubleMap()
			{
				return double_map;
			}

			map<string, string> getPrimitiveStringMap()
			{
				return string_map;
			}


		private:

			void generatePrimitiveData(string xml_path)
			{
				double_map = getPrimitiveMap<double>("double");
				string_map = getPrimitiveMap<string>("string");
				bool_map = getPrimitiveMap<bool>("bool");
				primitive_params = getPrimitiveParams();
			}

			template <typename data_type>
			map<string, data_type> getPrimitiveMap(string param_type)
			{
				if(xmlDoc.LoadFile(xml_path.c_str())== XML_SUCCESS)
				{
					XMLNode *primitive_defs;
					XMLNode *primitive;
					XMLNode *primitiveParam;

					map<string, data_type> primitive_map;

					/* Find primitive_defs node */
					primitive_defs = xmlDoc.FirstChildElement("main")->FirstChildElement("primitive-defs");
					if(primitive_defs)
					{
						/* Loop through primitive nodes */
						for (primitive = primitive_defs->FirstChildElement("primitive"); primitive != NULL; primitive = primitive->NextSiblingElement())
						{
							XMLElement *elem = primitive->ToElement();
							string primitiveName = elem->Attribute("name");
							//ROS_INFO("%s", primitiveName.c_str());

							for (primitiveParam = primitive->FirstChildElement("param"); primitiveParam != NULL; primitiveParam = primitiveParam->NextSiblingElement())
							{



								XMLElement *elem2 = primitiveParam->ToElement();
								string primitiveParamType = elem2->Attribute("type");
								if(primitiveParamType.compare(param_type)==0)
								{
									/*** Check if parameter is already present ***/
									if(primitive_map.find(elem2->GetText()) == primitive_map.end())
									{
										/*** Insert parameter ***/
										data_type *data_var = new data_type;
										primitive_map.insert(std::pair<string, data_type>(elem2->GetText(),*data_var));
									}
								}
							}
						}
					}
					return primitive_map;
				}
				else
				{
					ROS_FATAL("Mission execution: PRIMITIVE DEFINITION XML CANNOT BE OPENED.");
				}
			}


			map<int,std::vector<std::string> > getPrimitiveParams()
			{
				if(xmlDoc.LoadFile(xml_path.c_str())== XML_SUCCESS)
				{
					XMLNode *primitive_defs;
					XMLNode *primitive;
					XMLNode *primitiveParam;

					map<int,std::vector<std::string> > primitive_par;

					/* Find primitive_defs node */
					primitive_defs = xmlDoc.FirstChildElement("main")->FirstChildElement("primitive-defs");
					if(primitive_defs)
					{
						/* Loop through primitive nodes */
						for (primitive = primitive_defs->FirstChildElement("primitive"); primitive != NULL; primitive = primitive->NextSiblingElement())
						{
							XMLElement *elem = primitive->ToElement();
							string primitiveName = elem->Attribute("name");
							//ROS_INFO("%s", primitiveName.c_str());

							int primitive_id;
							for(primitive_id=0;primitive_id<static_cast<int>(primitiveNum);primitive_id++)
							{
								if(primitiveName.compare(PRIMITIVES[primitive_id])==0)
								{
									break;
								}
							}

					    	std::vector<std::string> tmp;

							for (primitiveParam = primitive->FirstChildElement("param"); primitiveParam != NULL; primitiveParam = primitiveParam->NextSiblingElement())
							{
								XMLElement *elem2 = primitiveParam->ToElement();
						    	tmp.push_back(elem2->GetText());
							}
							primitive_par.insert(std::pair<int,std::vector<std::string> >(primitive_id,tmp));
						}
					}
					return primitive_par;
				}
				else
				{
					ROS_FATAL("Mission execution: PRIMITIVE DEFINITION XML CANNOT BE OPENED.");
				}
			}

			XMLDocument xmlDoc;

			string xml_path;

			map<string, bool> bool_map;
			map<string, double> double_map;
			map<string, string> string_map;

		public:

			map<int,std::vector<std::string> > primitive_params;

		};
	}
}



#endif /* LABUST_ROS_PKG_LABUST_PRIMITIVES_INCLUDE_LABUST_PRIMITIVE_PRIMITIVEMAPGENERATOR_H_ */
