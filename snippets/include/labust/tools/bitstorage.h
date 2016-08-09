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
#ifndef SNIPPETS_BITSTORAGE_H
#define SNIPPETS_BITSTORAGE_H

#include <vector>
#include <cassert>
#include <cstdint>
#include <bitset>

namespace labust
{
	namespace tools
	{
		struct BitStorage
		{
			enum {TOP_STORE=256};

			static const uint64_t one = 1;

			BitStorage():_storage(0,0),bytept(0),bitpt(0)
			{
				_storage.reserve(TOP_STORE);
			};

			BitStorage(const std::vector<uint8_t>& storage):_storage(storage),bytept(0),bitpt(0){};

			///
			template<class ValueType>
			void put(const ValueType& data, double min, double max, uint8_t bitsz)
			{
				assert((min != max) && "Minimum and maximum should differ.");
				assert((bitsz > 0) && "Bit size should be larger than zero.");
				assert((min < max) && "Minimum should be smaller than maximum.");

				//Coerce to avoid overflow
				ValueType rdata = data;
				if (rdata > max) rdata = max; else if (rdata < min) rdata = min;
				//Get the size mask (maximum 64 bits for data representation)
				uint64_t mask = (one << bitsz)-one;
				//Normalize data
				uint64_t xw = uint64_t(round(mask * (rdata - min)/(max-min)));
				xw &= mask;
        //std::cout<<"\t Encoder number:"<<xw<<std::endl;

				uint8_t rembits(bitsz);
				while (rembits)
				{
          //introspect();
          //std::cout<<"\t rembits:"<<int(rembits)<<std::endl;
					//Check if enough size is available
					if (_storage.size() <= bytept) _storage.push_back(0);

					int rem = 8-(bitpt);
					if (rembits < rem) rem = rembits;
					_storage[bytept] |= uint8_t((xw >> (rembits - rem)) << (bitpt));
					bitpt += rem;
          //std::cout<<"\t rem:"<<int(rem)<<std::endl;
					if (bitpt == 8)
					{
						++(bytept);
						bitpt = 0;
					}
					rembits -= rem;
					xw &= (one << rembits) - one;
				}
        //introspect();
			}

			template<class ValueType>
			bool get(ValueType& data, double min, double max, uint8_t bitsz)
			{
				assert((min != max) && "Minimum and maximum should differ.");
				assert((bitsz > 0) && "Bit size should be larger than zero.");
				assert((min < max) && "Minimum should be smaller than maximum.");

				//Get the size mask (maximum 64 bits for data representation)
				uint64_t mask = (one << bitsz)-one;
				//Init data
				uint64_t xw(0);

				//Check if enough size is available
				if (((_storage.size() - bytept)*8-bitpt) < bitsz) return false;

				uint8_t rembits(bitsz);
				while (rembits)
				{
					int rem = 8-(bitpt);
					if (rembits < rem) rem = rembits;

					xw |= ((_storage[bytept] & (((one << rem) - one) << bitpt)) >> bitpt) << (rembits-rem);

					bitpt += rem;
					if (bitpt == 8)
					{
						++(bytept);
						bitpt = 0;
					}
					rembits -= rem;
				}

        //std::cout<<"\t Decoder number:"<<xw<<std::endl;

				data = ValueType(((max-min)*xw)/mask + min);
				return true;
			}

      /// Introspection into the storage
      void introspect()
      {
          // Introspection
          std::cout<<"BitStorage state"<<std::endl;
          std::cout<<"\t Size:"<<_storage.size()<<std::endl;
          std::cout<<"\t Bitpt:"<<int(bitpt)<<std::endl;
          std::cout<<"\t Bytept:"<<bytept<<std::endl;
          std::cout<<"\t Storage:";
          for (int i=0; i<_storage.size(); ++i)
          {
            std::cout<<std::bitset<8>(_storage[i])<<" ";
          }
          std::cout<<std::endl;
          usleep(2000*1000);
      }


			///Get storage
			const std::vector<uint8_t>& storage() const {return _storage;}

			///Reset storage
			void reset(){bytept=0; bitpt=0;}

			///Clear storage
			void clear(){_storage.clear();reset();}

			///Remaining bytes in storage
			int remaining(){return _storage.size() - bytept;}

		public:
			//The continuous byte storage
			std::vector<uint8_t> _storage;
			//The byte pointer
			uint32_t bytept;
			//The bit pointer
			uint32_t bitpt;
		};
	}
}
	/* SNIPPETS_BITSTORAGE_H */
#endif



