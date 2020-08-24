#pragma once
#ifndef HLL_OBJECT_H
#define HLL_OBJECT_H

#include <string>
#include <string.h>
#include "stdint.h"

class HllObject 
{
	char* hllObject;
	
	
	uint32_t totalCount;
	uint32_t skipBytesFront;
	uint32_t skipBytesBack;


	// temp
	void* mergeArray[2];
public:
	HllObject();
	HllObject(const HllObject& other);
	HllObject(uint32_t skipBytesFront, uint32_t skipBytesBack);
	HllObject& operator=(const HllObject& other);

	bool isInitialized() const;
	void init();


	~HllObject();

	uint64_t getHash(const char data[], int len);
	bool add(uint64_t hash);
	bool add(const char* data, int len);
	bool merge(const HllObject& other);
	std::string serialize();
	void deserialize(const char* data);
	uint64_t getCardinality();
	uint64_t getTotalProcessed() const { return totalCount; }
	void resetObject();

};

#endif
